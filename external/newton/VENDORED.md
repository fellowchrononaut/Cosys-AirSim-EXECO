# Newton — vendored, not a submodule

Upstream: https://github.com/newton-physics/newton.git
Commit:   6d3fdf6f  ("Honor --num-frames in the headless GL viewer", #3930)
Vendored: 2026-08-26, verified a pristine checkout with no local modifications.

⚠ **Vendored deliberately, so that a clone of this repository can run the MPM sidecar with nothing
fetched from elsewhere.** A submodule would have been smaller but is a pointer, not containment.
The `.git` directory is excluded — it was 124 MB of history against 41 MB of source.

⚠ **Upstream fixes are now a deliberate act.** There is no `git submodule update`. To re-vendor:

    git clone https://github.com/newton-physics/newton.git /tmp/newton
    cd /tmp/newton && git checkout <new-commit>
    rsync -a --delete --exclude='.git' --exclude='__pycache__' /tmp/newton/ external/newton/
    # then update the commit above, and re-run:
    #   python mpm_sidecar/warp_env_check.py     (sm_120 must still be native)
    #   python mpm_sidecar/wire_roundtrip_check.py

⚠ **Do not rely on a `.pth` install.** Before this was vendored, `newton` resolved through
`site-packages/newton.pth` pointing at a path outside any repository — so which Newton ran depended
on machine state rather than on the checkout. `sidecar.py` now puts this directory on `sys.path`
itself and reports which copy it loaded.
