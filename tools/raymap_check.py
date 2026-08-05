#!/usr/bin/env python3
"""Offline validation for the AirLib raymap builder (Phase 3b, step 1).

This is an INDEPENDENT re-implementation of the forward projection for each
camera model.  It never reads intrinsics out of the dump -- intrinsics come
from the original calibration file (Kalibr camchain / ScanNet++ transforms.json)
or from explicit command-line numbers.  The only thing taken from the dump is
the per-texel ray.

Conventions -- these must match AirLib/include/cameras/CameraModel.hpp:

  * Pixel centres are at INTEGER image coordinates.  Pixel (px, py) has its
    centre at continuous coordinates (u, v) = (px, py).  This is the OpenCV /
    Kalibr / ScanNet++ convention that cx, cy are expressed in.
  * Camera frame is the right-handed optical frame: +x right along a row,
    +y down a column, +z forward out of the lens.
  * Directions are unit length.  An all-zero direction marks a texel that the
    model cannot unproject (outside the model's valid domain).

Raymap dump format (little endian) -- see the same header for the C++ writer:

  off  size  content
    0     8  magic  "AIRRAYM1" (ASCII, no NUL terminator)
    8     4  uint32 version   = 1
   12     4  uint32 width
   16     4  uint32 height
   20     4  uint32 channels  = 6
   24     4  uint32 dtype     = 0 (float32) | 1 (float64)
   28     4  uint32 flags     = bit0 set -> central camera (all origins equal)
   32     4  uint32 reserved0 = 0
   36     4  uint32 reserved1 = 0
   40   ...  width*height*6 values, row major (y outer, x inner),
             per texel: ox, oy, oz, dx, dy, dz

Usage:
  raymap_check.py <dump> --model pinhole --fx .. --fy .. --cx .. --cy ..
  raymap_check.py <dump> --model kb --scannetpp <transforms.json>
  raymap_check.py <dump> --model ds --kalibr <camchain.yaml> [--cam cam0]
  raymap_check.py <dump> --model pinhole-fov --fov-degrees 90
  raymap_check.py <dump> --format-only
"""

import argparse
import json
import struct
import sys

import numpy as np

MAGIC = b"AIRRAYM1"
HEADER_BYTES = 40


# --------------------------------------------------------------------------
# dump reader
# --------------------------------------------------------------------------
def load_raymap(path):
    with open(path, "rb") as f:
        raw = f.read()
    if len(raw) < HEADER_BYTES:
        raise ValueError("file too short to hold a raymap header")
    magic = raw[:8]
    if magic != MAGIC:
        raise ValueError("bad magic %r (expected %r)" % (magic, MAGIC))
    (version, width, height, channels, dtype, flags,
     res0, res1) = struct.unpack("<8I", raw[8:HEADER_BYTES])
    if version != 1:
        raise ValueError("unsupported raymap version %d" % version)
    if channels != 6:
        raise ValueError("expected 6 channels per texel, header says %d" % channels)
    np_dtype = {0: "<f4", 1: "<f8"}[dtype]
    itemsize = np.dtype(np_dtype).itemsize
    expect = width * height * channels * itemsize
    body = raw[HEADER_BYTES:]
    if len(body) != expect:
        raise ValueError("payload is %d bytes, header implies %d" % (len(body), expect))
    data = np.frombuffer(body, dtype=np_dtype).reshape(height, width, channels)
    return {
        "version": version, "width": width, "height": height,
        "channels": channels, "dtype": dtype, "flags": flags,
        "reserved": (res0, res1),
        "origin": np.ascontiguousarray(data[:, :, 0:3]).astype(np.float64),
        "dir": np.ascontiguousarray(data[:, :, 3:6]).astype(np.float64),
    }


# --------------------------------------------------------------------------
# independent forward projections (camera frame -> pixel)
# --------------------------------------------------------------------------
def project_pinhole(pts, p):
    """Ideal pinhole.  pts: (N,3) in the optical frame."""
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    valid = z > 0
    with np.errstate(divide="ignore", invalid="ignore"):
        u = p["fx"] * x / z + p["cx"]
        v = p["fy"] * y / z + p["cy"]
    return u, v, valid


def project_kb_opencv(pts, p):
    """Kannala-Brandt via OpenCV's fisheye model (the OPENCV_FISHEYE of
    ScanNet++).  Deliberately a third-party implementation."""
    import cv2
    K = np.array([[p["fx"], 0.0, p["cx"]],
                  [0.0, p["fy"], p["cy"]],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    D = np.array([p["k1"], p["k2"], p["k3"], p["k4"]], dtype=np.float64)
    obj = np.ascontiguousarray(pts.reshape(-1, 1, 3))
    rvec = np.zeros(3, dtype=np.float64)
    tvec = np.zeros(3, dtype=np.float64)
    img, _ = cv2.fisheye.projectPoints(obj, rvec, tvec, K, D)
    img = img.reshape(-1, 2)
    valid = pts[:, 2] > 0
    return img[:, 0], img[:, 1], valid


def project_kb_scratch(pts, p):
    """From-scratch Kannala-Brandt, used as a cross-check on the OpenCV call."""
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    r = np.hypot(x, y)
    theta = np.arctan2(r, z)
    t2 = theta * theta
    td = theta * (1.0 + t2 * (p["k1"] + t2 * (p["k2"] + t2 * (p["k3"] + t2 * p["k4"]))))
    with np.errstate(divide="ignore", invalid="ignore"):
        s = np.where(r > 0, td / np.where(r > 0, r, 1.0), 0.0)
    u = p["fx"] * s * x + p["cx"]
    v = p["fy"] * s * y + p["cy"]
    # r == 0 -> the optical axis, projects to the principal point
    u = np.where(r > 0, u, p["cx"])
    v = np.where(r > 0, v, p["cy"])
    return u, v, np.ones_like(u, dtype=bool)


def project_ds(pts, p):
    """Double Sphere, written from the definition in Usenko, Demmel & Cremers,
    'The Double Sphere Camera Model' (3DV 2018), eqs. (41)-(45).

        d1 = ||(x,y,z)||
        d2 = ||(x, y, xi*d1 + z)||
        denom = alpha*d2 + (1-alpha)*(xi*d1 + z)
        u = fx * x / denom + cx
        v = fy * y / denom + cy

    valid iff z > -w2*d1, with
        w1 = alpha/(1-alpha)          if alpha <= 0.5
             (1-alpha)/alpha          otherwise
        w2 = (w1 + xi) / sqrt(2*w1*xi + xi^2 + 1)
    """
    xi, al = p["xi"], p["alpha"]
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    d1 = np.sqrt(x * x + y * y + z * z)
    zz = xi * d1 + z
    d2 = np.sqrt(x * x + y * y + zz * zz)
    denom = al * d2 + (1.0 - al) * zz
    w1 = al / (1.0 - al) if al <= 0.5 else (1.0 - al) / al
    w2 = (w1 + xi) / np.sqrt(2.0 * w1 * xi + xi * xi + 1.0)
    valid = (z > -w2 * d1) & (np.abs(denom) > 1e-12)
    with np.errstate(divide="ignore", invalid="ignore"):
        u = p["fx"] * x / denom + p["cx"]
        v = p["fy"] * y / denom + p["cy"]
    return u, v, valid


PROJECTORS = {
    "pinhole": project_pinhole,
    "pinhole-fov": project_pinhole,
    "kb": project_kb_opencv,
    "ds": project_ds,
}


# --------------------------------------------------------------------------
# calibration readers
# --------------------------------------------------------------------------
def params_from_scannetpp(path):
    with open(path) as f:
        j = json.load(f)
    if j.get("camera_model") != "OPENCV_FISHEYE":
        raise ValueError("expected camera_model OPENCV_FISHEYE, got %r"
                         % j.get("camera_model"))
    return {"fx": float(j["fl_x"]), "fy": float(j["fl_y"]),
            "cx": float(j["cx"]), "cy": float(j["cy"]),
            "k1": float(j["k1"]), "k2": float(j["k2"]),
            "k3": float(j["k3"]), "k4": float(j["k4"]),
            "width": int(j["w"]), "height": int(j["h"])}


def params_from_kalibr(path, cam="cam0"):
    import yaml
    with open(path) as f:
        y = yaml.safe_load(f)
    c = y[cam]
    if c.get("camera_model") != "ds":
        raise ValueError("expected camera_model ds, got %r" % c.get("camera_model"))
    xi, al, fx, fy, cx, cy = [float(v) for v in c["intrinsics"]]
    w, h = [int(v) for v in c["resolution"]]
    return {"xi": xi, "alpha": al, "fx": fx, "fy": fy, "cx": cx, "cy": cy,
            "width": w, "height": h}


def params_pinhole_from_fov(width, height, fov_degrees):
    """The intrinsics an ideal UE perspective capture of this size and
    horizontal FOV is equivalent to, under the integer-pixel-centre convention.

    UE maps NDC x in [-1,1] onto continuous viewport x in [0,W]; pixel px
    covers [px, px+1] so its centre sits at continuous x = px + 0.5.  Under the
    integer-pixel-centre convention the pixel coordinate is (continuous - 0.5),
    so NDC 0 lands on pixel coordinate W/2 - 0.5 = (W-1)/2.
    """
    fx = 0.5 * width / np.tan(0.5 * np.deg2rad(fov_degrees))
    return {"fx": fx, "fy": fx, "cx": 0.5 * (width - 1.0),
            "cy": 0.5 * (height - 1.0), "width": width, "height": height}


def ue_pinhole_dirs(width, height, fov_degrees):
    """Independent construction of the ray field of an ideal UE perspective
    capture, straight from NDC, with no intrinsics in between.  Used for the
    'pinhole must reduce to FOV_Degrees behaviour' check."""
    t = np.tan(0.5 * np.deg2rad(fov_degrees))
    px = np.arange(width, dtype=np.float64)
    py = np.arange(height, dtype=np.float64)
    gx, gy = np.meshgrid(px, py)
    ndc_x = 2.0 * (gx + 0.5) / width - 1.0
    ndc_y = 1.0 - 2.0 * (gy + 0.5) / height
    dx = ndc_x * t
    dy = -ndc_y * t * (height / width)  # optical +y is down, NDC +y is up
    dz = np.ones_like(dx)
    d = np.stack([dx, dy, dz], axis=-1)
    return d / np.linalg.norm(d, axis=-1, keepdims=True)


# --------------------------------------------------------------------------
# reporting
# --------------------------------------------------------------------------
def report_distribution(err, valid, width, height, params, label):
    """err: (H,W) pixel error.  Prints the full distribution plus the radial
    breakdown, because a wrong half-pixel convention is invisible at the
    centre and only shows at the edge."""
    gx, gy = np.meshgrid(np.arange(width, dtype=np.float64),
                         np.arange(height, dtype=np.float64))
    rad = np.hypot(gx - params["cx"], gy - params["cy"])
    rmax = rad.max()

    n_total = width * height
    n_valid = int(valid.sum())
    print("  %s" % label)
    print("    texels            : %d   valid %d (%.2f%%)  invalid %d"
          % (n_total, n_valid, 100.0 * n_valid / n_total, n_total - n_valid))
    if n_valid == 0:
        print("    NO VALID TEXELS")
        return None
    e = err[valid]
    print("    error px  mean %.3e  median %.3e  p99 %.3e  MAX %.3e"
          % (e.mean(), np.median(e), np.percentile(e, 99), e.max()))
    flat = np.where(valid, err, -np.inf)
    iy, ix = np.unravel_index(np.argmax(flat), flat.shape)
    print("    worst at pixel (x=%d, y=%d)  r=%.1f px  (%.1f%% of max radius "
          "%.1f px)" % (ix, iy, rad[iy, ix], 100.0 * rad[iy, ix] / rmax, rmax))
    print("    radial decile of r/rmax -> max error px (n valid):")
    for i in range(10):
        lo, hi = i / 10.0, (i + 1) / 10.0
        band = (rad >= lo * rmax) & (rad < hi * rmax if i < 9 else rad <= rmax)
        m = band & valid
        if m.sum() == 0:
            print("      [%.1f,%.1f)  -            (0)" % (lo, hi))
        else:
            print("      [%.1f,%.1f)  %.3e   (%d)" % (lo, hi, err[m].max(), int(m.sum())))
    return e.max()


def check_format(rm):
    print("  format")
    print("    %dx%d, %d channels, dtype %s, flags 0x%x"
          % (rm["width"], rm["height"], rm["channels"],
             "float32" if rm["dtype"] == 0 else "float64", rm["flags"]))
    o = rm["origin"].reshape(-1, 3)
    same = np.all(o == o[0])
    print("    origins identical across image: %s   value %s"
          % (same, tuple(o[0])))
    d = rm["dir"].reshape(-1, 3)
    n = np.linalg.norm(d, axis=1)
    nz = n > 0
    print("    directions: %d unit (max |‖d‖-1| = %.3e), %d zero (invalid)"
          % (int(nz.sum()), float(np.abs(n[nz] - 1.0).max()) if nz.any() else 0.0,
             int((~nz).sum())))
    return same


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dump")
    ap.add_argument("--model", choices=list(PROJECTORS.keys()) + ["none"],
                    default="none")
    ap.add_argument("--scannetpp")
    ap.add_argument("--kalibr")
    ap.add_argument("--cam", default="cam0")
    ap.add_argument("--fov-degrees", type=float)
    for k in ("fx", "fy", "cx", "cy", "k1", "k2", "k3", "k4", "xi", "alpha"):
        ap.add_argument("--" + k, type=float)
    ap.add_argument("--format-only", action="store_true")
    args = ap.parse_args()

    rm = load_raymap(args.dump)
    W, H = rm["width"], rm["height"]
    print("raymap %s" % args.dump)
    check_format(rm)
    if args.format_only or args.model == "none":
        return 0

    if args.scannetpp:
        p = params_from_scannetpp(args.scannetpp)
    elif args.kalibr:
        p = params_from_kalibr(args.kalibr, args.cam)
    elif args.model == "pinhole-fov":
        p = params_pinhole_from_fov(W, H, args.fov_degrees)
    else:
        p = {k: getattr(args, k) for k in
             ("fx", "fy", "cx", "cy", "k1", "k2", "k3", "k4", "xi", "alpha")
             if getattr(args, k) is not None}
        p["width"], p["height"] = W, H
    if p["width"] != W or p["height"] != H:
        print("  WARNING: calibration is %dx%d but dump is %dx%d"
              % (p["width"], p["height"], W, H))
    print("  calibration: %s" % {k: v for k, v in sorted(p.items())})

    d = rm["dir"].reshape(-1, 3)
    dump_valid = np.linalg.norm(d, axis=1) > 0

    u, v, model_valid = PROJECTORS[args.model](d, p)
    gx, gy = np.meshgrid(np.arange(W, dtype=np.float64),
                         np.arange(H, dtype=np.float64))
    err = np.hypot(u.reshape(H, W) - gx, v.reshape(H, W) - gy)
    valid = (dump_valid & model_valid & np.isfinite(u) & np.isfinite(v)).reshape(H, W)
    worst = report_distribution(err, valid, W, H, p, "round trip (%s)" % args.model)

    if args.model == "kb":
        u2, v2, _ = project_kb_scratch(d, p)
        e2 = np.hypot(u2.reshape(H, W) - gx, v2.reshape(H, W) - gy)
        print("    cross-check, from-scratch KB projection: max %.3e px"
              % e2[valid].max())
        print("    OpenCV vs from-scratch KB disagreement : max %.3e px"
              % np.hypot(u - u2, v - v2)[valid.reshape(-1)].max())

    if args.model == "pinhole-fov":
        ref = ue_pinhole_dirs(W, H, args.fov_degrees)
        dd = rm["dir"] .reshape(H, W, 3)
        ang = np.degrees(np.arccos(np.clip(np.sum(dd * ref, axis=-1), -1, 1)))
        # convert angular disagreement into pixels at the local focal length
        print("    vs independent UE-NDC ray field: max angle %.3e deg"
              " (= %.3e px at fx=%.3f)"
              % (ang.max(), np.deg2rad(ang.max()) * p["fx"], p["fx"]))
        # arccos(dot) is ill-conditioned for near-parallel unit vectors: an absolute
        # error of eps in the dot product shows up as sqrt(2*eps) ~ 1e-8 rad of
        # apparent angle.  2*asin(|a-b|/2) is exact in the same regime, so it is the
        # number that says whether the ray fields really differ.  Printed in addition
        # to the arccos figure, never instead of it.
        chord = np.linalg.norm(dd - ref, axis=-1)
        ang2 = 2.0 * np.arcsin(np.clip(0.5 * chord, 0.0, 1.0))
        print("    same, well-conditioned 2*asin(|d-ref|/2): max %.3e deg"
              " (= %.3e px at fx=%.3f)"
              % (np.degrees(ang2.max()), ang2.max() * p["fx"], p["fx"]))

    if worst is not None:
        print("  VERDICT: %s (worst %.3e px)"
              % ("SUB-PIXEL" if worst < 1.0 else "FAIL", worst))
    return 0


if __name__ == "__main__":
    sys.exit(main())
