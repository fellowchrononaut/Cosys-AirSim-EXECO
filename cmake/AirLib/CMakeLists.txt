cmake_minimum_required(VERSION 3.12.0)
project(AirLib)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules") 
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

IncludeEigen()

include_directories(
  ${AIRSIM_ROOT}/AirLib/
  ${AIRSIM_ROOT}/AirLib/include
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${RPC_LIB_INCLUDES}
  ${TINYXML2_INCLUDES}
)

# box3d headers only when the optional dependency is present.
if(BOX3D_FOUND)
  include_directories(${BOX3D_LIB_INCLUDES})
endif()

# MuJoCo headers likewise. The two backends are INDEPENDENT: either, both or neither may be built,
# and the choice is made per vehicle at runtime by the "PhysicsEngine" setting.
if(MUJOCO_FOUND)
  include_directories(${MUJOCO_LIB_INCLUDES})
endif()

# CoACD likewise. Unlike the two backends this is not an engine — it is a shared service BOTH of
# them use (UrdfConvexDecomposition.cpp), so it is guarded independently of either.
if(COACD_FOUND)
  include_directories(${COACD_LIB_INCLUDES})
endif()

file(GLOB_RECURSE ${PROJECT_NAME}_sources 
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/api/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/common/common_utils/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/physics/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/safety/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/car/api/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/computervision/api/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/multirotor/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/multirotor/api/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/urdfbot/api/*.cpp
)

# ⚠ Non-recursive GLOB on purpose. The call above is GLOB_RECURSE, so listing src/urdf/*.cpp there
# would also sweep in src/urdf/backends/, compiling the Box3D backend even when box3d is absent -
# which breaks the "byte-identical without the dependency" guarantee with a fatal
# 'box3d/box3d.h' file not found. The URDF model and parser have no box3d dependency and are
# always built; the backends are added below, only when the dependency is present.
file(GLOB ${PROJECT_NAME}_urdf_sources ${AIRSIM_ROOT}/${PROJECT_NAME}/src/urdf/*.cpp)
list(APPEND ${PROJECT_NAME}_sources ${${PROJECT_NAME}_urdf_sources})

# The Newton MPM sidecar link. Always built, and deliberately: it depends on nothing but POSIX
# mmap and the URDF descriptor headers, so it does not compromise the "byte-identical without the
# optional dependency" guarantee that governs the backends above. Newton itself is a separate
# PROCESS, not a library — there is nothing here to link against.
file(GLOB ${PROJECT_NAME}_mpm_sources ${AIRSIM_ROOT}/${PROJECT_NAME}/src/mpm/*.cpp)
list(APPEND ${PROJECT_NAME}_sources ${${PROJECT_NAME}_mpm_sources})

# tinyxml2, vendored as a single translation unit and compiled into AirLib rather than carried as
# a separate dependency. The URDF parser is the only consumer and it lives here; no plugin code
# includes it, so it needs no entry under AirLib/deps.
list(APPEND ${PROJECT_NAME}_sources ${AIRSIM_ROOT}/external/tinyxml2/tinyxml2.cpp)

# ⚠ -U_GNU_SOURCE is load-bearing, and removing it breaks the UNREAL link, not this build.
#
# build.sh compiles AirLib against the HOST glibc (2.39 here). Since glibc 2.38, <stdio.h> redirects
# sscanf to __isoc23_sscanf whenever __GLIBC_USE (ISOC2X) is on — which _GNU_SOURCE, defined by
# default in C++ mode, turns on. Unreal then links the resulting archive with its own bundled
# toolchain sysroot (v25_clang-18.1.0-rockylinux8, glibc 2.28), which has no such symbol:
#
#     ld.lld: error: undefined symbol: __isoc23_sscanf
#
# Undefining _GNU_SOURCE for this one file falls back to __isoc99_sscanf, present since glibc 2.7
# and therefore in Unreal's sysroot. Verified by inspecting the emitted symbol, not by assumption.
# tinyxml2 uses no GNU extensions, so this costs it nothing.
#
# ⚠ This is a targeted fix for the first symptom, NOT a general one. ANY AirLib translation unit
# compiled on a glibc >= 2.38 host that calls sscanf/strtol-family functions will fail the same way,
# and tinyxml2 merely happens to be the first one that ever did. The structural fix is to build
# AirLib against Unreal's bundled sysroot; that is a larger change and has not been made.
set_source_files_properties(${AIRSIM_ROOT}/external/tinyxml2/tinyxml2.cpp
  PROPERTIES COMPILE_OPTIONS "-U_GNU_SOURCE")

# The Box3D-backed URDF robot compiles only when the optional dependency is present.
# ⚠ GLOB, not GLOB_RECURSE. The Box3D sources sit directly in backends/; recursing would also sweep
# backends/mujoco/ into the BOX3D_FOUND guard, so a build with box3d but without MuJoCo would try to
# compile the MuJoCo backend and fail on a missing mujoco.h. The two engines are independent and
# their guards must be too.
if(BOX3D_FOUND)
  file(GLOB ${PROJECT_NAME}_box3d_sources
    ${AIRSIM_ROOT}/${PROJECT_NAME}/src/urdf/backends/*.cpp)
  list(APPEND ${PROJECT_NAME}_sources ${${PROJECT_NAME}_box3d_sources})
  add_definitions(-DWITH_BOX3D_BINDING=1)
else()
  add_definitions(-DWITH_BOX3D_BINDING=0)
endif()

# The MuJoCo-backed URDF robot, added 2026-08-18. Additive: Box3D is unaffected and remains the
# default. See external/mujoco/VENDORED.md and SIMVAL/urdf_physics/NEWTON-ASSESSMENT.md.
if(MUJOCO_FOUND)
  file(GLOB ${PROJECT_NAME}_mujoco_sources
    ${AIRSIM_ROOT}/${PROJECT_NAME}/src/urdf/backends/mujoco/*.cpp)
  list(APPEND ${PROJECT_NAME}_sources ${${PROJECT_NAME}_mujoco_sources})
  add_definitions(-DWITH_MUJOCO_BINDING=1)
else()
  add_definitions(-DWITH_MUJOCO_BINDING=0)
endif()

# ⚠ UrdfConvexDecomposition.cpp is compiled ALWAYS - it is in src/urdf/ and swept up by the glob
# above - and only its CoACD call is compiled out. That is deliberate: without the dependency it
# returns the input as a single convex part, which is exactly the behaviour every call site has
# today, so callers need no #if of their own and a build without CoACD is unchanged.
if(COACD_FOUND)
  add_definitions(-DWITH_COACD_BINDING=1)
else()
  add_definitions(-DWITH_COACD_BINDING=0)
endif()

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_sources})

# Standalone transaction/state-machine coverage for the engine-neutral coordinator. Keep it out of
# the default production build, but make it a first-class target and CTest without linking AirLib,
# Unreal, RPC, or any optional physics solver.
add_executable(PhysicsSceneCoordinatorTests EXCLUDE_FROM_ALL
  ${AIRSIM_ROOT}/AirLib/src/physics/PhysicsSceneCoordinator.cpp
  ${AIRSIM_ROOT}/AirLib/tests/PhysicsSceneCoordinatorTests.cpp
)
target_include_directories(PhysicsSceneCoordinatorTests PRIVATE
  ${AIRSIM_ROOT}/AirLib/include
)
add_test(NAME PhysicsSceneCoordinatorTests COMMAND PhysicsSceneCoordinatorTests)

CommonTargetLink()
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
if(MUJOCO_FOUND)
  target_link_libraries(${PROJECT_NAME} ${MUJOCO_LIB})
endif()
if(COACD_FOUND)
  target_link_libraries(${PROJECT_NAME} ${COACD_LIB})
endif()
target_link_libraries(${PROJECT_NAME} MavLinkCom)
if(BOX3D_FOUND)
  target_link_libraries(${PROJECT_NAME} ${BOX3D_LIB})
endif()

#string(SUBSTRING ${CMAKE_STATIC_LINKER_FLAGS} 9 -1 "BUILD_PLATFORM")
#find_package(Threads REQUIRED)
#target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
#target_link_libraries(${PROJECT_NAME} AirLib)
#target_link_libraries(${PROJECT_NAME} MavLinkCom)
#target_link_libraries(${PROJECT_NAME} ../AirLib/deps/rpclib/lib/${BUILD_PLATFORM}/$(Configuration)/rpc)
