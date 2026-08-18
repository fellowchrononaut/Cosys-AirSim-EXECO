#!/usr/bin/env bash
#
# Build the vendored C++ third-party dependencies against UNREAL'S bundled clang and sysroot, and
# stage them into AirLib/deps/ where build.sh and the UE plugin pick them up.
#
#   mujoco  — second URDF physics backend, alongside Box3D   (external/mujoco/VENDORED.md)
#   coacd   — approximate convex decomposition, used by BOTH  (external/coacd/VENDORED.md)
#
# ⚠ RUN THIS BEFORE ./build.sh, and only when you want those features. ./build.sh builds NEITHER.
# Without this step the plugin gets WITH_MUJOCO_BINDING=0 and WITH_COACD_BINDING=0: Box3D remains
# the only URDF backend and every mesh keeps the single-convex-hull behaviour it has today. That is
# the same behaviour as before either dependency existed, so nothing regresses by not running it.
#
# ⚠ WHY A SEPARATE BUILD AT ALL. One CMake tree has one compiler, and these need a different one
# from the rest of AirLib. Full argument in cmake/cmake-modules/UnrealToolchain.cmake; the short
# version is that the host glibc (>= 2.38) redirects sscanf/strtol to __isoc23_* symbols that
# Unreal's glibc 2.28 sysroot does not export, and unlike AirLib's tinyxml2 the redirect cannot be
# turned off with a flag, because libc++ itself requires _GNU_SOURCE. C dependencies (box3d) are
# unaffected, which is why this only appeared once a C++ one arrived.
#
# Usage:
#   ./build_thirdparty.sh                        # everything
#   ./build_thirdparty.sh --only coacd           # one of them
#   ./build_thirdparty.sh --engine /path/to/UE   # explicit engine root
#   ./build_thirdparty.sh --clean                # discard build_thirdparty/ first

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR" >/dev/null

engine_root="${UE_ROOT:-}"
clean=false
only=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --engine) engine_root="$2"; shift 2 ;;
        --only)   only="$2"; shift 2 ;;
        --clean)  clean=true; shift ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

want() { [[ -z "$only" || "$only" == "$1" ]]; }

# --- locate Unreal ----------------------------------------------------------------------------
if [[ -z "$engine_root" ]]; then
    for candidate in \
        "$HOME/Unreal_Engine"/UnrealEngine-*-release \
        "$HOME/UnrealEngine" \
        /opt/UnrealEngine ; do
        if [[ -d "$candidate/Engine" ]]; then engine_root="$candidate"; break; fi
    done
fi

if [[ ! -d "$engine_root/Engine" ]]; then
    echo "ERROR: could not find an Unreal Engine root." >&2
    echo "       Pass --engine <UE_ROOT>, or export UE_ROOT." >&2
    exit 1
fi

# ⚠ Glob rather than hardcode v25_clang-18.1.0-rockylinux8: Unreal ships one toolchain per engine
# version and the directory name changes with it. sort -V so v9 does not beat v25.
SDK_BASE="$engine_root/Engine/Extras/ThirdPartyNotUE/SDKs/HostLinux/Linux_x64"
UE_TOOLCHAIN="$(ls -d "$SDK_BASE"/v*_clang-*/x86_64-unknown-linux-gnu 2>/dev/null | sort -V | tail -1)"

if [[ -z "$UE_TOOLCHAIN" || ! -x "$UE_TOOLCHAIN/bin/clang++" ]]; then
    echo "ERROR: no v*_clang-*/x86_64-unknown-linux-gnu toolchain under $SDK_BASE" >&2
    echo "       Has the engine's Setup.sh been run?" >&2
    exit 1
fi
export UE_TOOLCHAIN

echo "engine    : $engine_root"
echo "toolchain : $UE_TOOLCHAIN"
"$UE_TOOLCHAIN/bin/clang++" --version | head -1
echo ""

build_dir=build_thirdparty
$clean && rm -rf "$build_dir"
mkdir -p "$build_dir"

# --- configure --------------------------------------------------------------------------------
# ⚠ Same source tree as build.sh, configured a second time. Only the third-party targets are built
# from it, so the other projects are configured and then ignored; that costs a second and keeps one
# definition of each dependency's option set (cmake/{mujoco,coacd}_wrapper) rather than a divergent
# copy here.
#
# ⚠ FETCHCONTENT_SOURCE_DIR_* point the dependencies-of-dependencies at the cache the ordinary
# build already populated, so a second tree does not re-clone them. Both MuJoCo and CoACD FETCH
# rather than vendor their own dependencies — see the VENDORED.md files, which record that as the
# weak point of both pins. CoACD's are the heavy ones: OpenVDB, Boost, TBB.
# ⚠ AT THE REPO ROOT, NOT UNDER cmake/. FetchContent puts each dependency's BUILD tree under this
# directory too, and OpenVDB's build configure_file()s a generated version.h into it. CMake REFUSES
# to configure a file into the current project's source directory, and the source directory here is
# `cmake` (see -S below) — so a cache under cmake/ dies with "attempted to configure a file into a
# source directory". MuJoCo's dependencies never noticed; OpenVDB's do.
cache="$SCRIPT_DIR/.fetchcontent-cache"
fc_args=("-DFETCHCONTENT_BASE_DIR=$cache")
for dep in ccd lodepng marchingcubecpp miniz qhull tinyobjloader tinyxml2 \
           openvdb boost tbb zlib spdlog eigen; do
    upper="$(echo "$dep" | tr '[:lower:]' '[:upper:]')"
    if [[ -d "$cache/$dep-src" ]]; then
        fc_args+=("-DFETCHCONTENT_SOURCE_DIR_$upper=$cache/$dep-src")
    fi
done

targets=()
cmake_opts=()
if want mujoco; then cmake_opts+=("-DAIRSIM_BUILD_MUJOCO=ON"); targets+=(mujoco); fi
if want coacd;  then cmake_opts+=("-DAIRSIM_BUILD_COACD=ON");  targets+=(coacd);  fi

if [[ ${#targets[@]} -eq 0 ]]; then
    echo "ERROR: --only '$only' matched nothing. Known: mujoco, coacd." >&2
    exit 2
fi

cmake -S cmake -B "$build_dir" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE="$SCRIPT_DIR/cmake/cmake-modules/UnrealToolchain.cmake" \
    "${cmake_opts[@]}" \
    "${fc_args[@]}"

for t in "${targets[@]}"; do
    cmake --build "$build_dir" --target "$t" -j"$(nproc)"
done

# --- merge, verify, stage ---------------------------------------------------------------------
# ⚠ A static build is MANY archives, not one. Shared linking absorbed the private dependencies into
# the .so; static exposes them — MuJoCo brings qhull, ccd, lodepng, miniz and tinyobjloader, CoACD
# brings OpenVDB, TBB, Boost and spdlog. The UE plugin's LoadOptionalAirSimDependency takes exactly
# one library name per dependency, so each is merged into a single self-contained archive rather
# than teaching Build.cs about a dependency graph it would then have to track across upgrades.
# ⚠ SEARCH the build tree rather than assuming one output directory. Three conventions collide
# here and none of them is ours to change: CommonSetup sends archives to output/lib, MuJoCo's own
# MujocoOptions.cmake redirects them to lib/, and TBB invents clang_18.1_cxx20_64_release/. Hunting
# for the file by name is the only thing that survives all three and any future fourth.
find_archive() {
    find "$build_dir" -name "lib$1.a" -type f 2>/dev/null | head -1
}

merge_and_stage() {
    local name="$1"; shift
    local primary="$1"; shift
    local parts=() found

    found="$(find_archive "$primary")"
    if [[ -z "$found" ]]; then
        echo "ERROR: lib$primary.a was not produced anywhere under $build_dir." >&2
        return 1
    fi
    parts+=("$found")

    # ⚠ A MISSING optional part is reported, not skipped silently. Each name here is a real link
    # dependency; if one stops being produced — renamed upstream, or quietly disabled by an option
    # — the merged archive is incomplete and the only symptom is an undefined symbol at UE link,
    # long after the cause.
    for a in "$@"; do
        found="$(find_archive "$a")"
        if [[ -n "$found" ]]; then parts+=("$found")
        else echo "  note: lib$a.a not produced - not merged"; fi
    done

    echo "$name: merging ${#parts[@]} archives"
    printf '  %s\n' "${parts[@]}"

    local merged="$build_dir/lib${name}_merged.a"
    rm -f "$merged"
    {
        echo "create $merged"
        for p in "${parts[@]}"; do echo "addlib $p"; done
        echo "save"
        echo "end"
    } | "$UE_TOOLCHAIN/bin/llvm-ar" -M

    # ⚠ Both checks exist because both failures are SILENT at build time and only surface at UE
    # link or editor startup. Cheap here, expensive there.
    #
    # 1. LTO bitcode. `ar` will happily archive LLVM IR; `file` still calls the result "current ar
    #    archive", and the plugin then fails to load with "file too short".
    local first tmpdir
    first="$("$UE_TOOLCHAIN/bin/llvm-ar" t "$merged" | head -1)"
    tmpdir="$(mktemp -d)"
    ( cd "$tmpdir" && "$UE_TOOLCHAIN/bin/llvm-ar" x "$merged" "$first" ) 2>/dev/null || true
    if [[ -f "$tmpdir/$first" ]] && ! file "$tmpdir/$first" | grep -q "ELF"; then
        echo "ERROR: '$first' is not an ELF object - LTO bitcode leaked in." >&2
        rm -rf "$tmpdir"; return 1
    fi
    rm -rf "$tmpdir"

    # 2. Post-2.28 glibc symbols. The whole reason this script uses Unreal's sysroot; if any appear
    #    the toolchain selection silently did not take effect.
    local isoc
    isoc=$(nm -u "$merged" 2>/dev/null | grep -c "__isoc2[0-9]_" || true)
    if [[ "$isoc" != "0" ]]; then
        echo "ERROR: $isoc __isoc2x_* symbols in lib$name.a - the UE sysroot did not apply." >&2
        nm -u "$merged" 2>/dev/null | grep "__isoc2[0-9]_" | sort -u >&2
        return 1
    fi

    mkdir -p "AirLib/deps/$name/lib"
    cp "$merged" "AirLib/deps/$name/lib/lib$name.a"
    echo "  -> AirLib/deps/$name/lib/lib$name.a  ($(du -h "$merged" | cut -f1))"
}

if want mujoco; then
    merge_and_stage mujoco mujoco ccd lodepng miniz qhullstatic_r render_noop tinyobjloader tinyxml2
    rsync -a --delete external/mujoco/include AirLib/deps/mujoco
fi

if want coacd; then
    # ⚠ OpenVDB and TBB are what WITH_3RD_PARTY_LIBS=ON buys, and they are not optional: without
    # them CoACD THROWS on a non-manifold mesh rather than degrading, and 9 of the Go2's 10 meshes
    # are non-manifold. Measured — see external/coacd/VENDORED.md.
    merge_and_stage coacd coacd openvdb tbb spdlog boost_iostreams boost_random
    mkdir -p AirLib/deps/coacd/include
    cp external/coacd/public/coacd.h AirLib/deps/coacd/include/coacd.h
fi

echo ""
echo "==============================================================="
echo " Staged. Now run ./build.sh to compile AirLib and the plugin"
echo " against these."
echo "==============================================================="

popd >/dev/null
