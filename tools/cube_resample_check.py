#!/usr/bin/env python3
"""Validation for the cube-capture resample pass (Phase 3b, step 4).

Three tests, in the order CAMERA-MODEL-DESIGN.md section 8 prescribes, because each
isolates one error source.  Run them in order; test 1 sets the noise floor for the
other two.

  test1   pinhole through the cube path vs a native pinhole render of the same pose.
          Both cameras carry the SAME intrinsics, one through cube capture + resample
          and one straight, so every difference is PURE RESAMPLING ERROR with no
          camera-model maths involved at all.

  test2   the phase gate.  Dense analytic projection: for each sample pixel of the
          generic camera, take the ray the calibration says that pixel looks along,
          project it into a native pinhole reference camera, and measure - to
          sub-pixel - how far the generic image's content actually sits from where
          the calibration puts it.  Reported separately at the centre and at the
          periphery, where the error concentrates.

          This uses the pinhole render as the oracle instead of hand-placed markers.
          It is denser and needs no detection, and the geometry it checks is the same:
          "the pixel the calibration says looks in direction d shows what is in
          direction d".

  depth   step 5's gate (T5.1).  Analytic geometry: for each output pixel take THAT
          pixel's own ray from the raymap, intersect it with a known plane, and compare
          with the returned value.  Reported RELATIVE, separately for centre, periphery
          and a band straddling a face boundary, and always next to the float16
          quantisation floor - an error below the format's own step is not a
          measurement.  Also runs the material check (is face depth range or planar?),
          the seam-continuity check (T5.2) and, given a reference pinhole, a
          same-origin range identity that needs no plane at all.

  seg     step 5's sharpest check (T5.3/T5.4).  Segmentation IDs are flat per-instance
          colours from a KNOWN 140-level lattice (PythonClient/cosysairsim/colormap.npy),
          so "did the resample invent an ID" is decidable with zero tolerance and no
          arguing.  Also measures how far an ID boundary sits from the corresponding
          Scene edge in the same camera.

  test3   seam check.  Classifies every output pixel by the cube face it came from,
          finds the boundaries, and compares the image gradient ACROSS a boundary
          with the gradient just inside each face.

          Read this before acting on test3 numbers:  ON MESH GEOMETRY a seam is a
          resampling bug, because each face is an exact pinhole render.  ON EWA /
          3DGS SPLATS A SEAM IS EXPECTED AND IS NOT A DEFECT - each face linearises
          the splat with its own pinhole projection, so a Gaussian straddling a
          boundary gets different 2D covariances either side (CAMERA-MODEL-DESIGN.md
          section 2.2; section 8 item 6 calls it "data, not a defect to chase").
          Validate on mesh; record the splat numbers as the EWA baseline.

The rays come from a raymap dump written by tools/raymap_dump.cpp out of the SAME
settings.json the simulator loaded, so this script never re-implements the camera
model - it checks the rendering against the rays the offline check in
tools/raymap_check.py already validated against OpenCV and Kalibr.

Usage:
  # build the dump first (from the repository root)
  g++ -std=c++17 -I AirLib/include -I AirLib/deps/eigen3 \
      tools/raymap_dump.cpp AirLib/src/common/common_utils/FileSystem.cpp \
      -o /tmp/raymap_dump
  /tmp/raymap_dump --settings ~/Documents/AirSim/settings.json \
      --camera fisheye_front --out /tmp/fisheye.raymap

  # then, with the simulator running.  BOTH of these first, every time:
  #   r.Tonemapper.Quality 0
  #   r.BloomQuality 0
  # Not optional since 2026-08-05.  APIPCamera::ensureFaceRig pins vignette, bloom,
  # chromatic aberration and film grain to zero on the CUBE FACES (they are per-view
  # radial effects and a face is not a view).  The reference is an ordinary AirSim
  # pinhole camera, so it still has all four.  Without these two commands the tests
  # measure the REFERENCE'S lens effects: test 1 reads 4.230 with them and 0.050
  # without, from the identical build.
  cube_resample_check.py test1 --vehicle CamPlayer --model-cam model_pinhole --ref-cam pinhole_ref
  cube_resample_check.py test2 --vehicle CamPlayer --model-cam fisheye_front --ref-cam pinhole_ref \
                               --raymap /tmp/fisheye.raymap
  cube_resample_check.py test3 --vehicle CamPlayer --model-cam fisheye_front \
                               --raymap /tmp/fisheye.raymap --label mesh-only
  cube_resample_check.py depth --vehicle CamPlayer --model-cam fisheye_front \
                               --raymap /tmp/fisheye.raymap --ref-cam ref_fwd \
                               --plane-normal 0,0,1 --plane-offset 1.85
  cube_resample_check.py seg   --vehicle CamPlayer --model-cam fisheye_front \
                               --ref-cam ref_fwd --extra-ref ref_left ref_right
  cube_resample_check.py normals --vehicle CamPlayer --model-cam fisheye_front \
                               --raymap /tmp/fisheye.raymap --ref-cam ref_fwd

WARM THE CAMERA.  Every subcommand here discards a throwaway capture first, and that is
not a nicety: the first capture after a rig is built reads against empty temporal
histories.  At step 4 that produced a mean of 9.17 where the warm answer was 1.047.
"""

import argparse
import os
import struct
import sys

import numpy as np

MAGIC = b"AIRRAYM1"
HEADER_BYTES = 40

# Face table.  Must stay identical to APIPCamera::getCubeFaceRotation and to
# CubeResample.usf - all three are the same six rows.  Vectors are in the UNREAL
# CAMERA frame: +X forward, +Y right, +Z up.
FACE_NAMES = ["Front", "Right", "Left", "Up", "Down", "Back"]


# --------------------------------------------------------------------------
# raymap dump reader (AIRRAYM1; the writer is AirLib/include/cameras/Raymap.hpp)
# --------------------------------------------------------------------------
def load_raymap(path):
    with open(path, "rb") as f:
        raw = f.read()
    if len(raw) < HEADER_BYTES or raw[:8] != MAGIC:
        raise ValueError("%s is not an AIRRAYM1 raymap dump" % path)
    (version, width, height, channels, dtype, flags,
     _r0, _r1) = struct.unpack("<8I", raw[8:HEADER_BYTES])
    if version != 1 or channels != 6:
        raise ValueError("unsupported raymap: version %d, %d channels" % (version, channels))
    np_dtype = {0: "<f4", 1: "<f8"}[dtype]
    data = np.frombuffer(raw[HEADER_BYTES:], dtype=np_dtype)
    data = data.reshape(height, width, channels).astype(np.float64)
    return {"width": width, "height": height, "central": bool(flags & 1), "data": data}


def optical_dirs(rm):
    """(H, W, 3) directions in the OPTICAL frame, plus a validity mask.

    Channels 0..2 are the origin.  This script does not read them, for the same
    reason CubeResample.usf does not: every v1 model is central, so all six cube
    faces share one origin and the cube path cannot express a per-pixel one.  The
    channels exist because ADR-001 says the format carries them.
    """
    d = rm["data"][:, :, 3:6]
    valid = np.linalg.norm(d, axis=2) > 1e-8
    return d, valid


def optical_to_unreal(d):
    """Optical (+x right, +y down, +z forward) -> Unreal camera (+X fwd, +Y right, +Z up).

    THE SAME change of basis APIPCamera::buildRaymapResource applies on the CPU before
    upload.  If these two ever disagree the image comes out mirrored or rotated, and
    this is one of the two places to look.
    """
    return np.stack([d[..., 2], d[..., 0], -d[..., 1]], axis=-1)


def optical_to_ned_body(d):
    """Optical -> AirSim body/NED axes (+x forward, +y right, +z down).

    Used only to compose with the camera orientation quaternions simGetImages returns,
    which are expressed in that frame.
    """
    return np.stack([d[..., 2], d[..., 0], d[..., 1]], axis=-1)


def ned_body_to_optical(d):
    return np.stack([d[..., 1], d[..., 2], d[..., 0]], axis=-1)


# --------------------------------------------------------------------------
# face selection - the shader's rule, in numpy
# --------------------------------------------------------------------------
def select_faces(d_ue):
    """(H, W) face index and (H, W, 2) face coordinates (a, b), both per the table
    in CubeResample.usf.  d_ue must already be in the Unreal camera frame."""
    x, y, z = d_ue[..., 0], d_ue[..., 1], d_ue[..., 2]
    ax, ay, az = np.abs(x), np.abs(y), np.abs(z)

    face = np.zeros(x.shape, dtype=np.int32)
    a = np.zeros(x.shape, dtype=np.float64)
    b = np.zeros(x.shape, dtype=np.float64)

    x_major = (ax >= ay) & (ax >= az)
    y_major = (~x_major) & (ay >= az)
    z_major = (~x_major) & (~y_major)

    def put(mask, index, num_a, num_b, den):
        sel = mask
        if not np.any(sel):
            return
        face[sel] = index
        with np.errstate(divide="ignore", invalid="ignore"):
            a[sel] = num_a[sel] / den[sel]
            b[sel] = num_b[sel] / den[sel]

    put(x_major & (x > 0), 0, y, z, x)        # Front: F=+X R=+Y U=+Z
    put(x_major & (x <= 0), 5, -y, z, -x)     # Back:  F=-X R=-Y U=+Z
    put(y_major & (y > 0), 1, -x, z, y)       # Right: F=+Y R=-X U=+Z
    put(y_major & (y <= 0), 2, x, z, -y)      # Left:  F=-Y R=+X U=+Z
    put(z_major & (z > 0), 3, y, -x, z)       # Up:    F=+Z R=+Y U=-X
    put(z_major & (z <= 0), 4, y, x, -z)      # Down:  F=-Z R=+Y U=+X

    return face, np.stack([a, b], axis=-1)


# --------------------------------------------------------------------------
# simulator access
# --------------------------------------------------------------------------
def connect(port=41451):
    try:
        import cosysairsim as airsim
    except ImportError:
        print("cosysairsim is not importable. Install PythonClient first:\n"
              "  pip install -e PythonClient", file=sys.stderr)
        raise
    # MultiAgent SimMode does not serve the single-vehicle default port: each vehicle gets
    # its own, so a rig that answers on 41453 is normal and 41451 simply times out.
    client = airsim.VehicleClient(port=port)
    client.confirmConnection()
    return client, airsim


def grab(client, airsim, vehicle, camera_names):
    """One simGetImages call for all cameras, so every image is the same sim instant
    (finding F9).  Returns name -> (H, W, 3) uint8 RGB, and name -> response."""
    requests = [airsim.ImageRequest(name, airsim.ImageType.Scene, False, False)
                for name in camera_names]
    responses = client.simGetImages(requests, vehicle_name=vehicle)
    images, metas = {}, {}
    for name, r in zip(camera_names, responses):
        if r.width == 0 or r.height == 0 or len(r.image_data_uint8) == 0:
            raise RuntimeError("camera %s returned no image (%s)" % (name, r.message))
        buf = np.frombuffer(r.image_data_uint8, dtype=np.uint8)
        images[name] = buf.reshape(r.height, r.width, 3).astype(np.float64)
        metas[name] = r
    return images, metas


def quat_matrix(q):
    """AirSim Quaternionr -> 3x3 rotation, body -> world."""
    w, x, y, z = q.w_val, q.x_val, q.y_val, q.z_val
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


# --------------------------------------------------------------------------
# small numeric helpers
# --------------------------------------------------------------------------
def luma(img):
    return 0.2126 * img[..., 0] + 0.7152 * img[..., 1] + 0.0722 * img[..., 2]


def sample_bilinear(img, u, v):
    """img (H, W, C) sampled at continuous pixel coordinates where integer
    coordinates are PIXEL CENTRES - the same convention AirLib's camera models use."""
    h, w = img.shape[:2]
    u = np.clip(u, 0.0, w - 1.0)
    v = np.clip(v, 0.0, h - 1.0)
    x0 = np.floor(u).astype(np.int64)
    y0 = np.floor(v).astype(np.int64)
    x1 = np.minimum(x0 + 1, w - 1)
    y1 = np.minimum(y0 + 1, h - 1)
    fx = (u - x0)[..., None]
    fy = (v - y0)[..., None]
    return (img[y0, x0] * (1 - fx) * (1 - fy) + img[y0, x1] * fx * (1 - fy) +
            img[y1, x0] * (1 - fx) * fy + img[y1, x1] * fx * fy)


def describe(diff, label):
    a = np.abs(diff)
    print("  %-22s mean %7.3f  median %7.3f  p95 %7.3f  p99 %7.3f  max %7.3f  "
          "exact %6.2f%%" %
          (label, a.mean(), np.median(a), np.percentile(a, 95), np.percentile(a, 99),
           a.max(), 100.0 * np.mean(a < 0.5)))


# --------------------------------------------------------------------------
# test 1 - pure resampling error
# --------------------------------------------------------------------------
def test1(args):
    client, airsim = connect(args.port)
    images, _ = grab(client, airsim, args.vehicle, [args.model_cam, args.ref_cam])
    a, b = images[args.model_cam], images[args.ref_cam]
    if a.shape != b.shape:
        raise SystemExit("images differ in size: %s vs %s. Both cameras must carry the "
                         "same Width/Height." % (a.shape, b.shape))

    diff = a - b
    print("test 1 - pinhole through the cube path vs native pinhole (%dx%d)" %
          (a.shape[1], a.shape[0]))
    print("  PURE RESAMPLING ERROR, 0-255 units. No camera-model maths is involved.")
    describe(diff, "all channels")
    describe(luma(a) - luma(b), "luma")
    # A near-constant signed offset is NOT resampling error: it is the tone mapper
    # reacting to the face render's different field of view. Separate it out rather
    # than letting it inflate the number this test exists to produce.
    mean_signed = np.mean(luma(a) - luma(b))
    print("  mean SIGNED luma difference %+7.3f  <- if this is large and the spread is "
          "small, fix exposure first (see the run recipe); it is tone mapping, not "
          "resampling" % mean_signed)


# --------------------------------------------------------------------------
# test 2 - dense analytic projection, the phase gate
# --------------------------------------------------------------------------
def subpixel_shift(actual, predicted, radius=4, min_texture=5.0):
    """Shift, in pixels, that best aligns predicted onto actual.  SSD over integer
    shifts then a parabola through the minimum in each axis.  Both patches are
    mean-subtracted and variance-normalised, so a brightness or exposure difference
    does not move the answer.

    min_texture is not a nicety.  Variance normalisation happily amplifies a patch whose
    whole contrast is sensor noise, and the SSD minimum then lands somewhere arbitrary -
    so a site with nothing in it does not fail, it returns a plausible wrong number.
    Measured 2026-08-05 on Blocks: sites with patch std 0.2-1.5 returned a median 1.4 px
    of pure noise, while sites with std > 30 in the same frame returned 0.081 px.  Scoring
    them together turned a passing gate into a failing one."""
    def norm(p):
        p = p - p.mean()
        s = p.std()
        return p / s if s > 1e-6 else None

    n = 2 * radius + 1
    inner = actual[radius:-radius, radius:-radius]
    if inner.std() < min_texture:
        return None  # too little contrast to locate anything
    a = norm(inner)
    if a is None:
        return None  # flat patch: carries no positional information at all

    best, best_ssd = None, None
    ssd = np.full((n, n), np.inf)
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            y0 = radius + dy
            x0 = radius + dx
            p = predicted[y0:y0 + inner.shape[0], x0:x0 + inner.shape[1]]
            pn = norm(p)
            if pn is None:
                continue
            value = np.mean((a - pn) ** 2)
            ssd[dy + radius, dx + radius] = value
            if best_ssd is None or value < best_ssd:
                best_ssd, best = value, (dy, dx)
    if best is None:
        return None
    dy, dx = best
    if abs(dy) == radius or abs(dx) == radius:
        return None  # the minimum ran off the search window: no reliable answer

    def refine(m0, m1, m2):
        # A neighbour of the minimum can still be inf: those shifts had a flat reference
        # patch and were skipped.  inf - inf is nan, and one nan poisons every statistic
        # downstream, so refuse to interpolate rather than propagate it.
        if not (np.isfinite(m0) and np.isfinite(m1) and np.isfinite(m2)):
            return 0.0
        denom = m0 - 2 * m1 + m2
        return 0.0 if abs(denom) < 1e-12 else 0.5 * (m0 - m2) / denom

    iy, ix = dy + radius, dx + radius
    sub_y = dy + refine(ssd[iy - 1, ix], ssd[iy, ix], ssd[iy + 1, ix])
    sub_x = dx + refine(ssd[iy, ix - 1], ssd[iy, ix], ssd[iy, ix + 1])
    return sub_x, sub_y


def test2(args):
    rm = load_raymap(args.raymap)
    d_opt, valid = optical_dirs(rm)
    h, w = rm["height"], rm["width"]

    client, airsim = connect(args.port)
    images, metas = grab(client, airsim, args.vehicle, [args.model_cam, args.ref_cam])
    model_img, ref_img = images[args.model_cam], images[args.ref_cam]
    if model_img.shape[:2] != (h, w):
        raise SystemExit("raymap is %dx%d but the camera returned %dx%d - the dump and "
                         "the running settings.json disagree" %
                         (w, h, model_img.shape[1], model_img.shape[0]))

    ref_h, ref_w = ref_img.shape[:2]
    ref_fx = (ref_w / 2.0) / np.tan(np.radians(args.ref_fov) / 2.0)
    ref_fy = ref_fx
    ref_cx, ref_cy = (ref_w - 1) / 2.0, (ref_h - 1) / 2.0

    # model camera optical -> world -> reference camera optical, from the poses the
    # response itself carries, so a mistyped rotation in settings.json cannot silently
    # become a projection error.
    r_model = quat_matrix(metas[args.model_cam].camera_orientation)
    r_ref = quat_matrix(metas[args.ref_cam].camera_orientation)
    r_model_to_ref = r_ref.T @ r_model

    patch = args.patch
    half = patch // 2
    radius = 4
    pad = half + radius

    # sample sites: a centre block plus rings out to the image edge, so "centre" and
    # "periphery" are reported separately rather than averaged into one number
    ys = np.linspace(pad, h - 1 - pad, args.grid).astype(int)
    xs = np.linspace(pad, w - 1 - pad, args.grid).astype(int)

    cx_img, cy_img = (w - 1) / 2.0, (h - 1) / 2.0
    max_r = np.hypot(cx_img, cy_img)

    results = []
    # Site accounting.  A gate that silently scores 4 sites out of 576 is not a gate, so
    # every rejection is counted and the reasons are printed with the verdict.
    reject = {"invalid ray": 0, "behind reference": 0, "outside reference": 0,
              "too little texture": 0}
    for y in ys:
        for x in xs:
            oy = slice(y - pad, y + pad + 1)
            ox = slice(x - pad, x + pad + 1)
            if not np.all(valid[oy, ox]):
                reject["invalid ray"] += 1
                continue

            d_ref = ned_body_to_optical(
                (r_model_to_ref @ optical_to_ned_body(d_opt[oy, ox]).reshape(-1, 3).T).T
            ).reshape(2 * pad + 1, 2 * pad + 1, 3)
            if np.any(d_ref[..., 2] <= 1e-6):
                reject["behind reference"] += 1
                continue  # outside the reference camera's hemisphere

            u = ref_fx * d_ref[..., 0] / d_ref[..., 2] + ref_cx
            v = ref_fy * d_ref[..., 1] / d_ref[..., 2] + ref_cy
            if u.min() < 0 or v.min() < 0 or u.max() > ref_w - 1 or v.max() > ref_h - 1:
                reject["outside reference"] += 1
                continue  # outside the reference camera's field of view

            predicted = luma(sample_bilinear(ref_img, u, v))
            actual = luma(model_img[oy, ox])
            shift = subpixel_shift(actual, predicted, radius=radius,
                                   min_texture=args.min_texture)
            if shift is None:
                reject["too little texture"] += 1
                continue
            dx, dy = shift
            r = np.hypot(x - cx_img, y - cy_img) / max_r
            results.append((r, np.hypot(dx, dy), dx, dy))

    if not results:
        raise SystemExit("no usable sample sites (%s). Widen --ref-fov, add reference "
                         "cameras pointing off-axis, or aim at a scene with more texture."
                         % ", ".join("%s %d" % kv for kv in reject.items()))

    results = np.array(results)
    print("test 2 - dense analytic projection against a native pinhole reference")
    print("  %d usable sites of %d, min texture %.1f LSB. Error in GENERIC-CAMERA PIXELS." %
          (len(results), args.grid * args.grid, args.min_texture))
    print("  rejected: " + ", ".join("%s %d" % kv for kv in reject.items()))
    for name, lo, hi in [("centre  (r<0.33)", 0.0, 0.33),
                         ("mid     (0.33-0.66)", 0.33, 0.66),
                         ("PERIPHERY (r>0.66)", 0.66, 1.01)]:
        sel = (results[:, 0] >= lo) & (results[:, 0] < hi)
        if not np.any(sel):
            print("  %-22s no sites" % name)
            continue
        e = results[sel, 1]
        print("  %-22s n=%4d  mean %6.3f px  median %6.3f px  p95 %6.3f px  max %6.3f px  %s" %
              (name, sel.sum(), e.mean(), np.median(e), np.percentile(e, 95), e.max(),
               "SUB-PIXEL" if np.median(e) < 1.0 else "NOT sub-pixel"))
    overall = np.median(results[:, 1])
    print("  gate: median error over all sites %.3f px -> %s" %
          (overall, "PASS" if overall < 1.0 else "FAIL"))



# --------------------------------------------------------------------------
# control - two native pinholes against each other, no camera model involved
# --------------------------------------------------------------------------
def control(args):
    """Test 2's noise floor.

    Runs test 2's exact machinery on two NATIVE PINHOLE cameras: rays of camera A,
    rotated by the two reported poses, projected into camera B.  No raymap, no cube
    capture, no resample - so whatever this reports is what the harness cannot resolve,
    and a test 2 arm is only meaningful to the extent it beats it.

    Measured 2026-08-05 on Blocks: ref_fwd vs ref_left 0.302 px, ref_fwd vs ref_up
    1.103 px.  The fisheye scored 0.36-0.48 and 1.29-1.45 against those same two
    references - i.e. it tracked the floor, and the arm that "failed" the gate failed
    against a reference that disagrees with another pinhole by just as much.  Run this
    for any reference before believing a test 2 failure.
    """
    client, airsim = connect(args.port)
    images, metas = grab(client, airsim, args.vehicle, [args.cam_a, args.cam_b])
    A, B = images[args.cam_a], images[args.cam_b]
    h, w = A.shape[:2]
    if B.shape[:2] != (h, w):
        raise SystemExit("both cameras must have the same Width/Height")
    fx = (w / 2.0) / np.tan(np.radians(args.fov) / 2.0)
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    r_a_to_b = quat_matrix(metas[args.cam_b].camera_orientation).T @ \
               quat_matrix(metas[args.cam_a].camera_orientation)

    radius = 5
    pad = args.patch // 2 + radius
    yy, xx = np.mgrid[0:2 * pad + 1, 0:2 * pad + 1].astype(np.float64)
    results = []
    for y in np.linspace(pad, h - 1 - pad, args.grid).astype(int):
        for x in np.linspace(pad, w - 1 - pad, args.grid).astype(int):
            d = np.stack([(x - pad + xx - cx) / fx, (y - pad + yy - cy) / fx,
                          np.ones_like(xx)], axis=-1)
            d /= np.linalg.norm(d, axis=-1, keepdims=True)
            d_b = ned_body_to_optical(
                (r_a_to_b @ optical_to_ned_body(d).reshape(-1, 3).T).T).reshape(d.shape)
            if np.any(d_b[..., 2] <= 1e-6):
                continue
            u = fx * d_b[..., 0] / d_b[..., 2] + cx
            v = fx * d_b[..., 1] / d_b[..., 2] + cy
            if u.min() < 0 or v.min() < 0 or u.max() > w - 1 or v.max() > h - 1:
                continue
            oy, ox = slice(y - pad, y + pad + 1), slice(x - pad, x + pad + 1)
            shift = subpixel_shift(luma(A[oy, ox]), luma(sample_bilinear(B, u, v)),
                                   radius=radius, min_texture=args.min_texture)
            if shift is None:
                continue
            results.append((np.hypot(*shift), shift[0], shift[1]))

    if not results:
        raise SystemExit("no overlapping textured sites for %s vs %s - the two frusta "
                         "may not overlap on anything with contrast" % (args.cam_a, args.cam_b))
    a = np.array(results)
    print("control - %s vs %s, both native pinholes. THE HARNESS NOISE FLOOR." %
          (args.cam_a, args.cam_b))
    print("  n=%d  median %.3f px  mean %.3f px  p95 %.3f px  mean dx %+.3f dy %+.3f" %
          (len(a), np.median(a[:, 0]), a[:, 0].mean(), np.percentile(a[:, 0], 95),
           a[:, 1].mean(), a[:, 2].mean()))
    print("  A test 2 arm against %s is only meaningful below this." % args.cam_b)


# --------------------------------------------------------------------------
# test 3 - seams in the output image
# --------------------------------------------------------------------------
def test3(args):
    rm = load_raymap(args.raymap)
    d_opt, valid = optical_dirs(rm)
    face, _ = select_faces(optical_to_unreal(d_opt))
    h, w = rm["height"], rm["width"]

    client, airsim = connect(args.port)
    images, _ = grab(client, airsim, args.vehicle, [args.model_cam])
    img = luma(images[args.model_cam])
    if img.shape != (h, w):
        raise SystemExit("raymap is %dx%d but the camera returned %dx%d" %
                         (w, h, img.shape[1], img.shape[0]))

    print("test 3 - face-boundary seam check, %s" % (args.label or "unlabelled scene"))
    print("  ON MESH a seam is a resampling bug. ON EWA/3DGS SPLATS a seam is EXPECTED")
    print("  (design section 2.2) - record it as the baseline, do not chase it.")

    # The comparison is a one-pixel step ACROSS a face boundary against the one-pixel
    # step everywhere else in the image - the same "cross vs within-face reference"
    # idea CubeFaceDump.cpp uses on the faces themselves, moved into output space,
    # where the boundaries are curves rather than straight edges.
    totals = {}
    interior_steps = []
    for axis in (0, 1):
        if axis == 0:
            fa, fb = face[:-1, :], face[1:, :]
            va, vb = valid[:-1, :], valid[1:, :]
            step = np.abs(img[:-1, :] - img[1:, :])
        else:
            fa, fb = face[:, :-1], face[:, 1:]
            va, vb = valid[:, :-1], valid[:, 1:]
            step = np.abs(img[:, :-1] - img[:, 1:])

        both_valid = va & vb
        boundary = (fa != fb) & both_valid
        interior_steps.append(step[(fa == fb) & both_valid])

        lo = np.minimum(fa, fb)
        hi = np.maximum(fa, fb)
        for f_lo in range(6):
            for f_hi in range(f_lo + 1, 6):
                sel = boundary & (lo == f_lo) & (hi == f_hi)
                if sel.sum() < 32:
                    continue
                totals.setdefault((f_lo, f_hi), []).append(step[sel])

    interior = np.concatenate(interior_steps)
    ref = np.median(interior)
    ref_p95 = np.percentile(interior, 95)
    print("  within-face reference: median one-pixel step %.3f, p95 %.3f (0-255 luma)" %
          (ref, ref_p95))

    worst = 0.0
    for key in sorted(totals):
        steps = np.concatenate(totals[key])
        med = np.median(steps)
        ratio = med / ref if ref > 1e-6 else float("inf")
        worst = max(worst, ratio)
        judgeable = ref > 0.1
        verdict = ("no gradient - not judged" if not judgeable
                   else ("consistent" if ratio <= 3.0 else "SEAM"))
        print("  %-14s n=%7d  median step %7.3f  p95 %7.3f  ratio %6.2f  %s" %
              ("%s|%s" % (FACE_NAMES[key[0]], FACE_NAMES[key[1]]), len(steps), med,
               np.percentile(steps, 95), ratio, verdict))
    print("  worst boundary/interior ratio %.2f" % worst)
    print("  no cause is diagnosed here: this measures agreement, not its explanation.")


# --------------------------------------------------------------------------
# step 5 - shared machinery for the modality checks
# --------------------------------------------------------------------------
IMAGE_TYPE_NAMES = ["Scene", "DepthPlanar", "DepthPerspective", "DepthVis",
                    "DisparityNormalized", "Segmentation", "SurfaceNormals", "Infrared",
                    "OpticalFlow", "OpticalFlowVis", "Lighting", "Annotation"]


def grab_typed(client, airsim, vehicle, specs):
    """specs is [(camera_name, image_type_name, as_float), ...].  ONE simGetImages call for
    all of them, so every image is the same sim instant (finding F9) - which is what makes
    "the segmentation edge and the Scene edge are in the same place" a meaningful question.

    Returns [(array, response)] in the order asked.  Float images come back (H, W) float64;
    uint8 images come back (H, W, 3) float64 RGB."""
    requests = [airsim.ImageRequest(name, getattr(airsim.ImageType, type_name), bool(as_float),
                                    False)
                for name, type_name, as_float in specs]
    responses = client.simGetImages(requests, vehicle_name=vehicle)
    out = []
    for (name, type_name, as_float), r in zip(specs, responses):
        if r.width == 0 or r.height == 0:
            raise RuntimeError("camera %s / %s returned no image (%s)" %
                               (name, type_name, r.message))
        if as_float:
            buf = np.array(r.image_data_float, dtype=np.float64)
            if buf.size != r.width * r.height:
                raise RuntimeError("camera %s / %s: %d floats for a %dx%d image" %
                                   (name, type_name, buf.size, r.width, r.height))
            out.append((buf.reshape(r.height, r.width), r))
        else:
            buf = np.frombuffer(r.image_data_uint8, dtype=np.uint8)
            if buf.size != r.width * r.height * 3:
                raise RuntimeError("camera %s / %s: %d bytes for a %dx%d RGB image" %
                                   (name, type_name, buf.size, r.width, r.height))
            out.append((buf.reshape(r.height, r.width, 3).astype(np.float64), r))
    return out


def warm(client, airsim, vehicle, specs):
    """Throwaway capture.  NOT optional - see the module docstring.  Also the call that
    builds the face rig for an ImageType that has never been requested, so the measured
    frame is never the one that paid for construction either."""
    try:
        grab_typed(client, airsim, vehicle, specs)
    except RuntimeError as exc:
        raise SystemExit("warm-up capture failed: %s" % exc)


def world_dirs_ned(d_opt, response):
    """Optical-frame ray directions -> world (NED) directions, using the pose the response
    itself carries.  Same composition test2 uses, so a mistyped rotation in settings.json
    cannot silently become a geometry error here either."""
    r_body_to_world = quat_matrix(response.camera_orientation)
    d_body = optical_to_ned_body(d_opt)
    flat = (r_body_to_world @ d_body.reshape(-1, 3).T).T
    return flat.reshape(d_opt.shape)


def float16_step(value):
    """The representable step of IEEE binary16 at `value`.  Quoted next to every depth
    number: an error smaller than the format's own quantisation is not a measurement.
    Depth render targets are InitAutoFormat -> PF_FloatRGBA and the readback is
    ReadFloat16Pixels, so this is the floor whatever the units turn out to be."""
    v = np.abs(np.asarray(value, dtype=np.float64))
    v = np.where(v < 6.104e-5, 6.104e-5, v)           # subnormal region
    exponent = np.floor(np.log2(v))
    return np.power(2.0, exponent - 10.0)             # 10 explicit mantissa bits


def radial_bins(h, w):
    """(H, W) normalised image radius, 0 at the centre and 1 at a corner."""
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float64)
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    return np.hypot(xx - cx, yy - cy) / np.hypot(cx, cy)


def seam_band(face, valid, width=3):
    """Pixels within `width` of a face boundary.  The band, not the boundary itself: the
    error a wrong conversion produces is largest just INSIDE each face, where the face
    coordinate is largest, and a one-pixel boundary mask would miss it."""
    edge = np.zeros(face.shape, dtype=bool)
    edge[:-1, :] |= (face[:-1, :] != face[1:, :])
    edge[1:, :] |= (face[:-1, :] != face[1:, :])
    edge[:, :-1] |= (face[:, :-1] != face[:, 1:])
    edge[:, 1:] |= (face[:, :-1] != face[:, 1:])
    band = edge.copy()
    for _ in range(max(0, width - 1)):
        grown = band.copy()
        grown[:-1, :] |= band[1:, :]
        grown[1:, :] |= band[:-1, :]
        grown[:, :-1] |= band[:, 1:]
        grown[:, 1:] |= band[:, :-1]
        band = grown
    return band & valid


def report_relative(err_rel, mask, label, floor_rel=None):
    sel = err_rel[mask & np.isfinite(err_rel)]
    if sel.size == 0:
        print("  %-24s no pixels" % label)
        return
    a = np.abs(sel)
    extra = ""
    if floor_rel is not None:
        extra = "   (float16 floor %.4f%%)" % (100.0 * floor_rel)
    print("  %-24s n=%8d  median %7.4f%%  p95 %7.4f%%  p99 %7.4f%%  max %8.4f%%%s" %
          (label, sel.size, 100.0 * np.median(a), 100.0 * np.percentile(a, 95),
           100.0 * np.percentile(a, 99), 100.0 * a.max(), extra))


def sobel_magnitude(img):
    """Gradient magnitude, used to compare an ID boundary with a Scene edge.  Correlating
    a segmentation image against a Scene image directly is meaningless - they carry
    unrelated values - but their EDGES are the same geometry, and that is what T5.4 asks
    about."""
    g = img if img.ndim == 2 else luma(img)
    gx = np.zeros_like(g)
    gy = np.zeros_like(g)
    gx[:, 1:-1] = g[:, 2:] - g[:, :-2]
    gy[1:-1, :] = g[2:, :] - g[:-2, :]
    return np.hypot(gx, gy)


# --------------------------------------------------------------------------
# step 5, depth  (T5.0 material question, T5.1 the gate, T5.2 seams)
# --------------------------------------------------------------------------
def depth_material_check(client, airsim, args):
    """T5.0 - IS FACE DEPTH RANGE FROM THE ORIGIN, OR PLANAR?  Step 5 section 3.3 turns on
    this and nothing else, so it runs first.

    It is answered on a NATIVE PINHOLE camera, with no cube path in play at all, and it
    needs no flat wall and no known geometry: DepthPlanar and DepthPerspective of the same
    pinhole pixel differ by exactly sqrt(1 + a^2 + b^2) if perspective is range and planar
    is planar, and by nothing at all if they are the same quantity.  Every pixel of any
    scene votes."""
    print("T5.0 - what does the depth material emit?  (native pinhole %s, no cube path)" %
          args.ref_cam)
    (planar, rp), (persp, _) = grab_typed(
        client, airsim, args.vehicle,
        [(args.ref_cam, "DepthPlanar", True), (args.ref_cam, "DepthPerspective", True)])
    if planar.shape != persp.shape:
        raise SystemExit("%s returns %s for DepthPlanar and %s for DepthPerspective - size "
                         "both CaptureSettings the same" % (args.ref_cam, planar.shape, persp.shape))

    h, w = planar.shape
    fx = (w / 2.0) / np.tan(np.radians(args.ref_fov) / 2.0)
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float64)
    a = (xx - cx) / fx
    b = (yy - cy) / fx
    predicted = np.sqrt(1.0 + a * a + b * b)

    good = (planar > 1e-3) & (persp > 1e-3) & np.isfinite(planar) & np.isfinite(persp)
    if not np.any(good):
        raise SystemExit("no finite non-zero depth pixels - is the camera pointing at the sky?")
    ratio = persp[good] / planar[good]

    err_range = np.abs(ratio - predicted[good]) / predicted[good]
    err_same = np.abs(ratio - 1.0)
    print("  ratio DepthPerspective/DepthPlanar, over %d pixels" % ratio.size)
    print("    median ratio %.5f   (1.0 = the two are the same quantity;"
          " %.5f = perspective is range)" % (np.median(ratio), np.median(predicted[good])))
    print("    vs sqrt(1+a^2+b^2):  median relative residual %.4f%%" % (100 * np.median(err_range)))
    print("    vs 1:                median relative residual %.4f%%" % (100 * np.median(err_same)))
    verdict = ("PERSPECTIVE IS RANGE FROM THE ORIGIN, PLANAR IS PLANAR"
               if np.median(err_range) < np.median(err_same)
               else "the two materials emit the SAME quantity - re-read section 3.3")
    print("    -> %s" % verdict)

    # the corner is where the two hypotheses are furthest apart, so quote the number there
    corner = predicted[good].max()
    print("    the two hypotheses differ by %.3fx at the widest pixel of this camera, so this "
          "is not a marginal call" % corner)

    # units.  Scale is invisible to the ratio above, so ask separately.
    print("  units: median DepthPerspective over the frame %.3f, min %.3f, max %.3f" %
          (np.median(persp[good]), persp[good].min(), persp[good].max()))
    print("    AirSim documents depth in METRES (docs/image_apis.md). If these read ~100x a "
          "plausible distance in metres, the material is emitting centimetres and every "
          "relative number below is still valid while every absolute one is not.")
    return rp


def depth(args):
    rm = load_raymap(args.raymap)
    d_opt, valid = optical_dirs(rm)
    h, w = rm["height"], rm["width"]
    face, _ = select_faces(optical_to_unreal(d_opt))

    client, airsim = connect(args.port)

    specs = [(args.model_cam, args.image_type, True),
             (args.model_cam, "DepthPerspective", True)]
    if args.ref_cam:
        specs += [(args.ref_cam, "DepthPlanar", True), (args.ref_cam, "DepthPerspective", True)]
    warm(client, airsim, args.vehicle, specs)

    if args.ref_cam:
        depth_material_check(client, airsim, args)
        print()

    grabbed = grab_typed(client, airsim, args.vehicle,
                         [(args.model_cam, args.image_type, True),
                          (args.model_cam, "DepthPerspective", True)])
    (img, meta), (persp_img, _) = grabbed
    if img.shape != (h, w):
        raise SystemExit("raymap is %dx%d but %s/%s returned %dx%d. The generic camera's "
                         "CaptureSettings for this ImageType must match its Scene block - "
                         "UnrealImageCapture refuses the cube path otherwise and you are "
                         "looking at a pinhole image." %
                         (w, h, args.model_cam, args.image_type, img.shape[1], img.shape[0]))

    # The blendable swap of section 3.3, checked rather than assumed: on a generic camera
    # DepthPlanar and DepthPerspective are deliberately the same image.
    same = np.mean(img == persp_img)
    print("generic-camera DepthPlanar vs DepthPerspective: %.2f%% of pixels bit-identical" %
          (100.0 * same))
    print("  100%% is the intended result (design section 5 O3). Anything else means the "
          "DepthPlanar face captures did not get the perspective material.")
    print()

    finite = valid & np.isfinite(img) & (img > 1e-6)
    if not np.any(finite):
        raise SystemExit("no usable depth pixels")
    floor_rel = float(np.median(float16_step(img[finite]) / img[finite]))
    print("float16 quantisation floor at this frame's ranges: %.4f%% relative "
          "(median step %.5f in the value's own units at a median range of %.3f)" %
          (100.0 * floor_rel, float(np.median(float16_step(img[finite]))),
           float(np.median(img[finite]))))
    print()

    # ---------------- T5.1, the gate: analytic plane ----------------
    n = np.array([float(v) for v in args.plane_normal.split(",")], dtype=np.float64)
    n /= np.linalg.norm(n)
    d_world = world_dirs_ned(d_opt, meta)
    p = np.array([meta.camera_position.x_val, meta.camera_position.y_val,
                  meta.camera_position.z_val], dtype=np.float64)
    denom = d_world @ n
    toward = finite & (denom > args.min_cos)

    if not np.any(toward):
        print("T5.1 - no rays reach the plane (normal %s). Point the camera at it, or pass a "
              "different --plane-normal." % args.plane_normal)
    else:
        # Two arms.  With --plane-offset the test is ABSOLUTE and also checks the units and
        # any scale error.  Without it, the offset is fitted from the rays most nearly
        # perpendicular to the plane - where the geometry is best conditioned - and what is
        # left is a pure SHAPE test: does one scalar explain the whole surface, periphery
        # and face boundaries included?  A planar-instead-of-range error fails the shape
        # test badly and cannot hide in the offset.
        if args.plane_offset is not None:
            offset = float(args.plane_offset)
            arm = "absolute (--plane-offset %.4f)" % offset
        else:
            best = toward & (denom >= np.percentile(denom[toward], 95))
            offset = float(np.median(p @ n + img[best] * denom[best]))
            arm = "shape only (offset fitted to %.4f from the %d best-conditioned rays)" % (
                offset, int(best.sum()))

        predicted = (offset - p @ n) / denom
        ok = toward & (predicted > 0)
        err_rel = np.where(ok, (img - predicted) / np.maximum(predicted, 1e-9), np.nan)

        r = radial_bins(h, w)
        band = seam_band(face, ok, width=args.seam_width)
        print("T5.1 - depth against analytic geometry, %s. THE STEP GATE." % arm)
        print("  plane normal %s in NED, camera at (%.3f, %.3f, %.3f)" %
              (np.array2string(n, precision=3), p[0], p[1], p[2]))
        report_relative(err_rel, ok & (r < 0.33), "centre  (r<0.33)", floor_rel)
        report_relative(err_rel, ok & (r >= 0.33) & (r < 0.66), "mid     (0.33-0.66)", floor_rel)
        report_relative(err_rel, ok & (r >= 0.66), "PERIPHERY (r>0.66)", floor_rel)
        report_relative(err_rel, band & ok, "seam band (+/-%dpx)" % args.seam_width, floor_rel)
        overall = np.nanmedian(np.abs(err_rel[ok]))
        print("  gate: median relative error over the plane %.4f%% -> %s" %
              (100.0 * overall, "PASS" if overall < 0.01 else "FAIL"))
        print("  the periphery is where a wrong conversion shows: planar-instead-of-range is "
              "off by sqrt(1+a^2+b^2), which is 1.000 at a face centre and 1.732 at a corner.")
    print()

    # ---------------- T5.2, seam continuity, in the value's own units ----------------
    print("T5.2 - depth continuity across face boundaries, ON MESH.")
    print("  Faces share one origin, so range is a CONTINUOUS function of direction: a step "
          "at a seam is a bug, not a property. (On splats it is expected - section 4.1.)")
    interior, boundary = [], []
    for axis in (0, 1):
        if axis == 0:
            fa, fb = face[:-1, :], face[1:, :]
            va, vb = finite[:-1, :], finite[1:, :]
            step = np.abs(img[:-1, :] - img[1:, :])
            scale = 0.5 * (img[:-1, :] + img[1:, :])
        else:
            fa, fb = face[:, :-1], face[:, 1:]
            va, vb = finite[:, :-1], finite[:, 1:]
            step = np.abs(img[:, :-1] - img[:, 1:])
            scale = 0.5 * (img[:, :-1] + img[:, 1:])
        both = va & vb
        rel = step / np.maximum(scale, 1e-9)
        interior.append(rel[(fa == fb) & both])
        boundary.append(rel[(fa != fb) & both])
    interior = np.concatenate(interior)
    boundary = np.concatenate(boundary)
    if boundary.size == 0:
        print("  no face boundaries in this image")
    else:
        ref = np.median(interior)
        med = np.median(boundary)
        print("  one-pixel relative step: within a face median %.4f%% p95 %.4f%%" %
              (100 * ref, 100 * np.percentile(interior, 95)))
        print("  one-pixel relative step: across a boundary median %.4f%% p95 %.4f%%  (n=%d)" %
              (100 * med, 100 * np.percentile(boundary, 95), boundary.size))
        print("  ratio %.2f -> %s" % (med / ref if ref > 1e-12 else float("inf"),
                                      "consistent" if ref > 1e-12 and med <= 3.0 * ref else "SEAM"))
        print("  in absolute units, median step across a boundary %.5f" %
              float(np.median(np.abs(np.diff(img, axis=1))[face[:, :-1] != face[:, 1:]])))
    print()

    # ---------------- cross-check against a native pinhole, no plane needed ----------
    if args.ref_cam:
        print("cross-check - same-origin range identity against the native pinhole %s." %
              args.ref_cam)
        print("  Both cameras sit at one point, so the range along a given WORLD direction is "
              "the same number for both. No geometry is assumed; the renderer is the oracle, "
              "so this cannot catch an error the two paths share - it catches the ones step 5 "
              "introduces.")
        (ref_img, ref_meta), = grab_typed(client, airsim, args.vehicle,
                                          [(args.ref_cam, "DepthPerspective", True)])
        rh, rw = ref_img.shape
        rfx = (rw / 2.0) / np.tan(np.radians(args.ref_fov) / 2.0)
        rcx, rcy = (rw - 1) / 2.0, (rh - 1) / 2.0
        r_model_to_ref = quat_matrix(ref_meta.camera_orientation).T @ \
            quat_matrix(meta.camera_orientation)
        d_ref = ned_body_to_optical(
            (r_model_to_ref @ optical_to_ned_body(d_opt).reshape(-1, 3).T).T).reshape(d_opt.shape)
        inside = finite & (d_ref[..., 2] > 1e-6)
        u = np.where(inside, rfx * d_ref[..., 0] / np.maximum(d_ref[..., 2], 1e-9) + rcx, 0)
        v = np.where(inside, rfx * d_ref[..., 1] / np.maximum(d_ref[..., 2], 1e-9) + rcy, 0)
        inside &= (u >= 1) & (v >= 1) & (u <= rw - 2) & (v <= rh - 2)
        if not np.any(inside):
            print("  no overlap between the two frusta")
        else:
            ref_at = sample_bilinear(ref_img[..., None], u, v)[..., 0]
            err = np.where(inside, (img - ref_at) / np.maximum(ref_at, 1e-9), np.nan)
            r = radial_bins(h, w)
            report_relative(err, inside & (r < 0.33), "centre  (r<0.33)", floor_rel)
            report_relative(err, inside & (r >= 0.33) & (r < 0.66), "mid     (0.33-0.66)", floor_rel)
            report_relative(err, inside & (r >= 0.66), "PERIPHERY (r>0.66)", floor_rel)
            print("  NOTE this arm bilinearly samples the reference, so it inherits the "
                  "reference's own resampling error at every silhouette. Read the median, "
                  "not the max.")


# --------------------------------------------------------------------------
# step 5, segmentation  (T5.3 ID exactness, T5.4 alignment with Scene)
# --------------------------------------------------------------------------
def segmentation_levels():
    """The 140 values per channel that a Cosys-AirSim instance-segmentation colour can take.

    Cosys-AirSim does NOT render segmentation through a palette material: APIPCamera calls
    FObjectAnnotator::SetViewForAnnotationRender and draws per-instance AnnotationComponents
    as flat unlit colours, drawn from colormap.npy.  So the legal ID set is a known 140^3
    lattice, the same file the client ships, and "did the resample invent an ID" has an
    exact answer with no tolerance to argue about."""
    candidates = []
    try:
        import cosysairsim
        candidates.append(os.path.join(os.path.dirname(os.path.abspath(cosysairsim.__file__)),
                                       "colormap.npy"))
    except ImportError:
        pass
    #repo-relative fallback: this file lives in tools/
    candidates.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir,
                                   "PythonClient", "cosysairsim", "colormap.npy"))
    colormap = None
    for path in candidates:
        if os.path.exists(path):
            colormap = np.load(path)
            break
    if colormap is None:
        raise SystemExit("cannot find the instance-segmentation colormap. It ships as "
                         "PythonClient/cosysairsim/colormap.npy.")
    levels = np.unique(colormap[:, 0])
    legal = np.zeros(256, dtype=bool)
    legal[levels.astype(np.int64)] = True
    return legal, colormap


def novel_id_report(img, legal, label):
    """Pixels whose colour is NOT on the lattice.  Black is counted apart: it is the
    background of an annotation render, and it is also what the resample writes for a ray
    the camera model could not unproject, so it is legal by construction."""
    q = np.rint(img).astype(np.int64)
    black = np.all(q == 0, axis=-1)
    on_lattice = legal[q[..., 0]] & legal[q[..., 1]] & legal[q[..., 2]]
    novel = ~on_lattice & ~black
    colours = set(map(tuple, q[novel].reshape(-1, 3))) if np.any(novel) else set()
    total = set(map(tuple, q.reshape(-1, 3)))
    print("  %-16s %8d px novel of %8d  (%.4f%%)   %5d distinct colours, %4d of them novel, "
          "%7d px background" %
          (label, int(novel.sum()), q.shape[0] * q.shape[1],
           100.0 * novel.mean(), len(total), len(colours), int(black.sum())))
    return int(novel.sum()), len(colours), total


def seg(args):
    client, airsim = connect(args.port)

    specs = [(args.model_cam, "Segmentation", False), (args.model_cam, "Scene", False)]
    refs = [c for c in ([args.ref_cam] if args.ref_cam else []) + list(args.extra_ref or [])]
    specs += [(c, "Segmentation", False) for c in refs]
    warm(client, airsim, args.vehicle, specs)
    grabbed = grab_typed(client, airsim, args.vehicle, specs)

    seg_img, seg_meta = grabbed[0]
    scene_img, _ = grabbed[1]
    ref_imgs = {c: grabbed[2 + i][0] for i, c in enumerate(refs)}

    legal, _ = segmentation_levels()

    print("T5.3 - segmentation ID exactness. BINARY: any novel ID is a failure.")
    print("  Legal IDs are the 140-value-per-channel lattice of colormap.npy. Bilinear "
          "between two instance colours lands off it; nearest cannot.")
    novel, _, model_set = novel_id_report(seg_img, legal, args.model_cam)
    for c in refs:
        novel_id_report(ref_imgs[c], legal, c + " (control)")
    if refs:
        print("  the control is a NATIVE PINHOLE through the same readback and the same uint8 "
              "conversion. If it is not zero, the readback is the suspect, not the resample.")
    print("  T5.3 verdict: %s (%d novel pixels)" % ("PASS" if novel == 0 else "FAIL", novel))

    if refs:
        union = set()
        for c in refs:
            q = np.rint(ref_imgs[c]).astype(np.int64)
            union |= set(map(tuple, q.reshape(-1, 3)))
        extra = model_set - union
        print("  informational: %d of the generic camera's %d distinct colours are absent from "
              "the union of %s. The references do not tile the whole fisheye field of view, so "
              "a non-zero count here is expected and is NOT the gate." %
              (len(extra), len(model_set), ", ".join(refs)))

    # per-pixel oracle, available only when the generic camera is a MODEL PINHOLE matched to
    # a native one (the settings_step4_test1_reg rig). Then there is a right answer per pixel.
    for c in refs:
        if ref_imgs[c].shape == seg_img.shape:
            exact = np.mean(np.all(np.rint(seg_img) == np.rint(ref_imgs[c]), axis=-1))
            print("  per-pixel vs %s (same size): %.3f%% of pixels bit-identical. Meaningful "
                  "ONLY if that camera is a native pinhole at the same pose as a model-pinhole "
                  "generic camera; otherwise ignore it." % (c, 100.0 * exact))

    print()
    print("T5.4 - does an ID boundary land on the Scene edge? Same camera, same rays, so the "
          "expected shift is zero.")
    seg_edge = sobel_magnitude(seg_img)
    scene_edge = sobel_magnitude(scene_img)
    if seg_edge.shape != scene_edge.shape:
        print("  Scene is %s and Segmentation is %s - size both CaptureSettings the same" %
              (scene_edge.shape, seg_edge.shape))
        return
    h, w = seg_edge.shape
    radius = 4
    pad = args.patch // 2 + radius
    results = []
    rejected = 0
    for y in np.linspace(pad, h - 1 - pad, args.grid).astype(int):
        for x in np.linspace(pad, w - 1 - pad, args.grid).astype(int):
            oy, ox = slice(y - pad, y + pad + 1), slice(x - pad, x + pad + 1)
            shift = subpixel_shift(seg_edge[oy, ox], scene_edge[oy, ox], radius=radius,
                                   min_texture=args.min_texture)
            if shift is None:
                rejected += 1
                continue
            results.append(np.hypot(*shift))
    if not results:
        raise SystemExit("no usable sites - aim at a scene with object boundaries in it")
    a = np.array(results)
    print("  n=%d sites of %d (%d rejected for too little edge), median %.3f px  p95 %.3f px "
          "-> %s" % (len(a), args.grid * args.grid, rejected, np.median(a),
                     np.percentile(a, 95), "PASS" if np.median(a) < 1.0 else "FAIL"))
    print("  the harness noise floor for a shift like this was measured at 1.103 px between "
          "two NATIVE pinholes at step 4 - run `control` before believing a failure.")


# --------------------------------------------------------------------------
# step 5, surface normals  (T5.5 - unit length, and the FRAME question of section 3.4)
# --------------------------------------------------------------------------
def normals(args):
    """Two questions, and the second answers section 3.4 without needing a hand-placed
    surface.

    1. Is |n| = 1 after decode?  Blending four unit vectors and forgetting to renormalise
       leaves a shortfall that is largest exactly where the normal changes fastest, so this
       is checked over the whole image and not on average.

    2. IS THE FRAME COMMON ACROSS FACES?  If AirSim's normals were view space they would be
       per-face, and two faces looking at ONE flat surface would encode different vectors -
       by up to a 90 degree rotation, i.e. |dn| of order 1 across a face boundary.  Comparing
       the cross-boundary step with the within-face step therefore measures the frame
       directly.  NormalsMaterial reads the scene texture PPI_WorldNormal, which is the
       GBuffer's WORLD normal, so the expected answer is "common" - but that is read off an
       asset, and this is the measurement."""
    rm = load_raymap(args.raymap)
    d_opt, valid = optical_dirs(rm)
    h, w = rm["height"], rm["width"]
    face, _ = select_faces(optical_to_unreal(d_opt))

    client, airsim = connect(args.port)
    specs = [(args.model_cam, "SurfaceNormals", False)]
    if args.ref_cam:
        specs.append((args.ref_cam, "SurfaceNormals", False))
    warm(client, airsim, args.vehicle, specs)
    grabbed = grab_typed(client, airsim, args.vehicle, specs)
    img, _ = grabbed[0]
    if img.shape[:2] != (h, w):
        raise SystemExit("raymap is %dx%d but %s returned %dx%d - size the SurfaceNormals "
                         "CaptureSettings the same as Scene, or the camera silently stays on "
                         "the pinhole path" % (w, h, args.model_cam, img.shape[1], img.shape[0]))

    def decode(rgb):
        return rgb / 255.0 * 2.0 - 1.0

    n = decode(img)
    length = np.linalg.norm(n, axis=-1)
    # A never-written or out-of-domain pixel decodes to (-1,-1,-1); exclude it rather than let
    # it dominate a percentile.
    written = valid & (np.any(img > 0.5, axis=-1))
    print("T5.5a - |n| after decode, over %d written pixels" % int(written.sum()))
    dev = np.abs(length[written] - 1.0)
    print("  median |1 - |n|| %.5f   p95 %.5f   p99 %.5f   max %.5f" %
          (np.median(dev), np.percentile(dev, 95), np.percentile(dev, 99), dev.max()))
    print("  the readback is 8 bit per channel, so one LSB is 2/255 = %.5f on each component "
          "and |n| carries about that as its own floor. Anything an order of magnitude above "
          "it is a missing renormalisation, not quantisation." % (2.0 / 255.0))
    if args.ref_cam:
        ref = decode(grabbed[1][0])
        ref_written = np.any(grabbed[1][0] > 0.5, axis=-1)
        ref_dev = np.abs(np.linalg.norm(ref, axis=-1)[ref_written] - 1.0)
        print("  control, native pinhole %s: median %.5f   p95 %.5f" %
              (args.ref_cam, np.median(ref_dev), np.percentile(ref_dev, 95)))

    print()
    print("T5.5b - is the normal frame common across faces? (section 3.4)")
    interior, boundary = [], []
    for axis in (0, 1):
        if axis == 0:
            fa, fb = face[:-1, :], face[1:, :]
            ok = written[:-1, :] & written[1:, :]
            step = np.linalg.norm(n[:-1, :] - n[1:, :], axis=-1)
        else:
            fa, fb = face[:, :-1], face[:, 1:]
            ok = written[:, :-1] & written[:, 1:]
            step = np.linalg.norm(n[:, :-1] - n[:, 1:], axis=-1)
        interior.append(step[(fa == fb) & ok])
        boundary.append(step[(fa != fb) & ok])
    interior = np.concatenate(interior)
    boundary = np.concatenate(boundary)
    if boundary.size == 0:
        print("  no face boundaries with written pixels either side")
        return
    ref_step = np.median(interior)
    med = np.median(boundary)
    print("  one-pixel |dn|: within a face median %.5f  p95 %.5f  (n=%d)" %
          (ref_step, np.percentile(interior, 95), interior.size))
    print("  one-pixel |dn|: across a boundary median %.5f  p95 %.5f  (n=%d)" %
          (med, np.percentile(boundary, 95), boundary.size))
    print("  ratio %.2f" % (med / ref_step if ref_step > 1e-9 else float("inf")))
    print("  READ IT LIKE THIS: face-space normals would step by up to |dn| = 2 across a "
          "boundary - a 90 degree rotation of the frame - so a cross-boundary median in the "
          "same order as the within-face one means WORLD SPACE and no rotation is needed. A "
          "median of order 0.5 or more means face space: set "
          "airsim.CubeResampleNormalsFrame 1, re-derive the encoding convention in "
          "CubeResample.usf, and re-run.")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="test", required=True)

    def common(p):
        p.add_argument("--vehicle", default="CamPlayer")
        p.add_argument("--port", type=int, default=41451,
                       help="RPC port. MultiAgent gives each vehicle its own; "
                            "the CamPlayer rig used for these tests answers on 41453")
        p.add_argument("--model-cam", required=True,
                       help="camera carrying the CameraModel block")

    p1 = sub.add_parser("test1", help="pure resampling error")
    common(p1)
    p1.add_argument("--ref-cam", required=True, help="native pinhole camera, same pose")
    p1.set_defaults(func=test1)

    p2 = sub.add_parser("test2", help="dense analytic projection (the gate)")
    common(p2)
    p2.add_argument("--ref-cam", required=True)
    p2.add_argument("--ref-fov", type=float, default=90.0,
                    help="FOV_Degrees of the reference camera, from settings.json")
    p2.add_argument("--raymap", required=True)
    p2.add_argument("--grid", type=int, default=24)
    p2.add_argument("--patch", type=int, default=25)
    p2.add_argument("--min-texture", type=float, default=5.0,
                    help="minimum patch std (0-255 luma) for a site to be scored. Below "
                         "this the correlation locks onto noise and returns a confident "
                         "wrong shift - it is what made this gate read FAIL on Blocks")
    p2.set_defaults(func=test2)

    pc = sub.add_parser("control", help="test 2's noise floor: two native pinholes")
    pc.add_argument("--vehicle", default="CamPlayer")
    pc.add_argument("--port", type=int, default=41451)
    pc.add_argument("--cam-a", required=True)
    pc.add_argument("--cam-b", required=True)
    pc.add_argument("--fov", type=float, default=90.0)
    pc.add_argument("--grid", type=int, default=40)
    pc.add_argument("--patch", type=int, default=25)
    pc.add_argument("--min-texture", type=float, default=10.0)
    pc.set_defaults(func=control)

    p3 = sub.add_parser("test3", help="face-boundary seam check")
    common(p3)
    p3.add_argument("--raymap", required=True)
    p3.add_argument("--label", default="", help="e.g. 'mesh-only' or 'splats'")
    p3.set_defaults(func=test3)

    pd = sub.add_parser("depth", help="step 5 gate: depth against analytic geometry")
    common(pd)
    pd.add_argument("--raymap", required=True)
    pd.add_argument("--image-type", default="DepthPlanar", choices=IMAGE_TYPE_NAMES)
    pd.add_argument("--ref-cam", default=None,
                    help="native pinhole at the same pose. Enables the T5.0 material check "
                         "and the same-origin cross-check; both are skipped without it")
    pd.add_argument("--ref-fov", type=float, default=90.0)
    pd.add_argument("--plane-normal", default="0,0,1",
                    help="analytic plane's unit normal in NED (default 0,0,1 = a horizontal "
                         "floor, since NED +z is DOWN)")
    pd.add_argument("--plane-offset", type=float, default=None,
                    help="n.X of the plane, in the depth image's own units. Omit to FIT it "
                         "from the best-conditioned rays, which turns the gate into a pure "
                         "shape test that no offset or scale error can hide in")
    pd.add_argument("--min-cos", type=float, default=0.15,
                    help="reject rays that graze the plane; below this the intersection is "
                         "ill-conditioned and the residual is about the plane, not the camera")
    pd.add_argument("--seam-width", type=int, default=3)
    pd.set_defaults(func=depth)

    ps = sub.add_parser("seg", help="step 5: segmentation ID exactness and alignment")
    common(ps)
    ps.add_argument("--ref-cam", default=None, help="native pinhole, used as the ID control")
    ps.add_argument("--extra-ref", nargs="*", default=[],
                    help="further native pinholes, to widen the union the fisheye is "
                         "compared against")
    ps.add_argument("--grid", type=int, default=24)
    ps.add_argument("--patch", type=int, default=25)
    ps.add_argument("--min-texture", type=float, default=8.0,
                    help="minimum patch std of the EDGE map for a T5.4 site to be scored")
    ps.set_defaults(func=seg)

    pn = sub.add_parser("normals", help="step 5: |n| = 1 and the normals frame")
    common(pn)
    pn.add_argument("--raymap", required=True)
    pn.add_argument("--ref-cam", default=None, help="native pinhole, for the |n| control")
    pn.set_defaults(func=normals)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    sys.exit(main() or 0)
