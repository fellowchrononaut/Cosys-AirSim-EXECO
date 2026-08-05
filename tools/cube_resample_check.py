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

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    sys.exit(main() or 0)
