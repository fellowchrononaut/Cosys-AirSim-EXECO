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

file(GLOB_RECURSE ${PROJECT_NAME}_sources 
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/api/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/common/common_utils/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/safety/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/car/api/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/computervision/api/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/multirotor/*.cpp
  ${AIRSIM_ROOT}/${PROJECT_NAME}/src/vehicles/multirotor/api/*.cpp
)

# ⚠ Non-recursive GLOB on purpose. The call above is GLOB_RECURSE, so listing src/urdf/*.cpp there
# would also sweep in src/urdf/backends/, compiling the Box3D backend even when box3d is absent -
# which breaks the "byte-identical without the dependency" guarantee with a fatal
# 'box3d/box3d.h' file not found. The URDF model and parser have no box3d dependency and are
# always built; the backends are added below, only when the dependency is present.
file(GLOB ${PROJECT_NAME}_urdf_sources ${AIRSIM_ROOT}/${PROJECT_NAME}/src/urdf/*.cpp)
list(APPEND ${PROJECT_NAME}_sources ${${PROJECT_NAME}_urdf_sources})

# tinyxml2, vendored as a single translation unit and compiled into AirLib rather than carried as
# a separate dependency. The URDF parser is the only consumer and it lives here; no plugin code
# includes it, so it needs no entry under AirLib/deps.
list(APPEND ${PROJECT_NAME}_sources ${AIRSIM_ROOT}/external/tinyxml2/tinyxml2.cpp)

# The Box3D-backed URDF robot compiles only when the optional dependency is present.
if(BOX3D_FOUND)
  file(GLOB_RECURSE ${PROJECT_NAME}_box3d_sources
    ${AIRSIM_ROOT}/${PROJECT_NAME}/src/urdf/backends/*.cpp)
  list(APPEND ${PROJECT_NAME}_sources ${${PROJECT_NAME}_box3d_sources})
  add_definitions(-DWITH_BOX3D_BINDING=1)
else()
  add_definitions(-DWITH_BOX3D_BINDING=0)
endif()

add_library(${PROJECT_NAME} STATIC ${${PROJECT_NAME}_sources})

CommonTargetLink()
target_link_libraries(${PROJECT_NAME} ${RPC_LIB})
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
