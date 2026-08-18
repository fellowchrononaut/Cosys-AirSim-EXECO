# tinyxml2, vendored

Source: https://github.com/leethomason/tinyxml2

## Upgraded to 11.0.0 on 2026-08-18

Was 10.0.0. Raised because **MuJoCo fetches tinyxml2 11.0.0 for itself**, and two major versions
cannot coexist in one binary: `libAirLib.a` carried v10 symbols while `libmujoco.a` called
`XMLPrinter::XMLPrinter(FILE*, bool, int, EscapeAposCharsInAttributes)` — a v11 signature — giving
an undefined reference at final link. The AirSim plugin links both into one `.so`, so one version
has to win.

Upgrading ours rather than pinning MuJoCo back: MuJoCo's own source uses the v11 API, so it cannot
build against v10. Our use is the basic document/element interface, which is unchanged across the
two.
