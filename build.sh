#!/usr/bin/env bash

# get path of current script: https://stackoverflow.com/a/39340259/207661
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
pushd "$SCRIPT_DIR"  >/dev/null

set -e
set -x

debug=false
gcc=false
# Parse command line arguments
while [[ $# -gt 0 ]]
do
    key="$1"

    case $key in
    --debug)
        debug=true
        shift # past argument
        ;;
    --gcc)
        gcc=true
        shift # past argument
        ;;
    esac

done

function version_less_than_equal_to() { test "$(printf '%s\n' "$@" | sort -V | head -n 1)" = "$1"; }

# check for rpclib
RPC_VERSION_FOLDER="rpclib-2.3.1"
if [ ! -d "./external/rpclib/$RPC_VERSION_FOLDER" ]; then
    echo "ERROR: new version of AirSim requires newer rpclib."
    echo "please run setup.sh first and then run build.sh again."
    exit 1
fi

# check for local cmake build created by setup.sh
if [ -d "./cmake_build" ]; then
    if [ "$(uname)" == "Darwin" ]; then
        CMAKE="$(greadlink -f cmake_build/bin/cmake)"
    else
        CMAKE="$(readlink -f cmake_build/bin/cmake)"
    fi
else
    CMAKE=$(which cmake)
fi

# variable for build output
if $debug; then
    build_dir=build_debug
else
    build_dir=build_release
fi 
if [ "$(uname)" == "Darwin" ]; then
    # llvm v8 is too old for Big Sur see
    # https://github.com/microsoft/AirSim/issues/3691
    #export CC=/usr/local/opt/llvm@8/bin/clang
    #export CXX=/usr/local/opt/llvm@8/bin/clang++
    #now pick up whatever setup.sh installs
    export CC="$(brew --prefix)/opt/llvm/bin/clang"
    export CXX="$(brew --prefix)/opt/llvm/bin/clang++"
else
    if $gcc; then
        export CC="gcc"
        export CXX="g++"
    else
        export CC="clang"
        export CXX="clang++"
    fi
fi

#install EIGEN library
if [[ ! -d "./AirLib/deps/eigen3/Eigen" ]]; then
    echo "### Eigen is not installed. Please run setup.sh first."
    exit 1
fi

echo "putting build in $build_dir folder, to clean, just delete the directory..."

# this ensures the cmake files will be built in our $build_dir instead.
if [[ -f "./cmake/CMakeCache.txt" ]]; then
    rm "./cmake/CMakeCache.txt"
fi
if [[ -d "./cmake/CMakeFiles" ]]; then
    rm -rf "./cmake/CMakeFiles"
fi



if [[ ! -d $build_dir ]]; then
    mkdir -p $build_dir
fi

pushd $build_dir  >/dev/null
if $debug; then
    folder_name="Debug"
    "$CMAKE" ../cmake -DCMAKE_BUILD_TYPE=Debug $CMAKE_VARS \
        || (popd && rm -r $build_dir && exit 1)   
else
    folder_name="Release"
    "$CMAKE" ../cmake -DCMAKE_BUILD_TYPE=Release $CMAKE_VARS \
        || (popd && rm -r $build_dir && exit 1)
fi
popd >/dev/null


pushd $build_dir  >/dev/null
# final linking of the binaries can fail due to a missing libc++abi library
# (happens on Fedora, see https://bugzilla.redhat.com/show_bug.cgi?id=1332306).
# So we only build the libraries here for now
make -lc++fs -j"$(nproc)"
popd >/dev/null

mkdir -p AirLib/lib/x64/$folder_name
mkdir -p AirLib/deps/rpclib/lib
mkdir -p AirLib/deps/MavLinkCom/lib
cp $build_dir/output/lib/libAirLib.a AirLib/lib
cp $build_dir/output/lib/libMavLinkCom.a AirLib/deps/MavLinkCom/lib
cp $build_dir/output/lib/librpc.a AirLib/deps/rpclib/lib/librpc.a

# box3d is OPTIONAL: absent => the UE plugin gets WITH_BOX3D_BINDING=0 and compiles it out.
if [ -f "$build_dir/output/lib/libbox3d.a" ]; then
    mkdir -p AirLib/deps/box3d/lib
    cp $build_dir/output/lib/libbox3d.a AirLib/deps/box3d/lib/libbox3d.a
fi

# MuJoCo is OPTIONAL, INDEPENDENT of box3d, and NOT BUILT HERE.
#
# It is built out-of-tree by ./build_thirdparty.sh, against Unreal's bundled clang and sysroot rather
# than the host's - one CMake tree has one compiler, and MuJoCo needs a different one. The reason
# is in cmake/cmake-modules/UnrealToolchain.cmake: on a glibc >= 2.38 host the C++ standard headers
# redirect sscanf/strtol to __isoc23_* symbols that Unreal's glibc 2.28 sysroot does not export,
# and unlike AirLib's tinyxml2 the redirect cannot be switched off, because libc++ itself requires
# _GNU_SOURCE.
#
# So there is nothing to copy here. build_thirdparty.sh stages AirLib/deps/mujoco/{lib,include}
# directly, and the rsync below carries it into the plugin like any other dependency. If it was
# never run, AirLib/deps/mujoco does not exist, WITH_MUJOCO_BINDING=0, and Box3D is the only URDF
# backend - exactly as before MuJoCo was vendored.

# Update AirLib/lib, AirLib/deps, Plugins folders with new binaries
rsync -a --delete $build_dir/output/lib/ AirLib/lib/x64/$folder_name
rsync -a --delete external/rpclib/$RPC_VERSION_FOLDER/include AirLib/deps/rpclib
if [ -d "external/box3d/include" ]; then
    rsync -a --delete external/box3d/include AirLib/deps/box3d
fi
rsync -a --delete MavLinkCom/include AirLib/deps/MavLinkCom
rsync -a --delete AirLib Unreal/Plugins/AirSim/Source
rm -rf Unreal/Plugins/AirSim/Source/AirLib/src

set +x

echo ""
echo ""
echo "==============================="
echo " Cosys-AirSim plugin is built!."
echo "==============================="
echo ""
echo "For further info see for installation see:"
echo "https://github.com/Cosys-Lab/Cosys-AirSim/tree/main/docs/install_linux.md"
echo "=================================================================="

popd >/dev/null
