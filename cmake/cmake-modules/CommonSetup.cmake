# Common setup instructions shared by all AirSim CMakeLists.

macro(CommonTargetLink)
    target_link_libraries(${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})
    #target_link_libraries(c++abi)
endmacro(CommonTargetLink)

macro(IncludeEigen)
    include_directories(${AIRSIM_ROOT}/AirLib/deps/eigen3)
endmacro(IncludeEigen)

macro(AddExecutableSource)
    set(PROJECT_CPP ${PROJECT_NAME}_sources)
    file(GLOB_RECURSE PROJECT_CPP "${AIRSIM_ROOT}/${PROJECT_NAME}/*.cpp")
    add_executable(${PROJECT_NAME} ${PROJECT_CPP})
endmacro(AddExecutableSource)

macro(SetupConsoleBuild)
    IF(UNIX)
    ELSE()
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_CONSOLE ")
        set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:CONSOLE")
    ENDIF()
endmacro(SetupConsoleBuild)

macro(CommonSetup)
    find_package(Threads REQUIRED)
    #⚠ NO_CMAKE_FIND_ROOT_PATH, and it is load-bearing for the third-party build. That build uses
    #cmake-modules/UnrealToolchain.cmake, which confines find_* to Unreal's sysroot so no host
    #library can be linked into the plugin (see the find-root note there — find_package(ZLIB) was
    #caught reaching into /usr/lib). find_path obeys the same confinement, so without this opt-out
    #it goes looking for OUR OWN AirSim.sln inside the sysroot and fails with
    #"Could not find AIRSIM_ROOT". This is locating the repository, not a dependency, so it is
    #exactly the case the opt-out is for.
    find_path(AIRSIM_ROOT NAMES AirSim.sln PATHS ".." "../.." "../../.." "../../../.." "../../../../.." "../../../../../.." REQUIRED NO_CMAKE_FIND_ROOT_PATH)

    #setup output paths
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib)
    SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/output/bin)
    SET(LIBRARY_OUTPUT_PATH ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})

    #setup include and lib for rpclib which will be referenced by other projects
    set(RPCLIB_VERSION_FOLDER rpclib-2.3.1)
    set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/${RPCLIB_VERSION_FOLDER}/include")
    #name of .a file with lib prefix
    set(RPC_LIB rpc)

    #setup include and lib for box3d (URDF robot dynamics backend), which is OPTIONAL.
    #Absent => BOX3D_FOUND is FALSE, nothing links it, and WITH_BOX3D_BINDING=0 compiles it out.
    if(EXISTS "${AIRSIM_ROOT}/external/box3d/CMakeLists.txt")
        set(BOX3D_FOUND TRUE)
        set(BOX3D_LIB_INCLUDES "${AIRSIM_ROOT}/external/box3d/include")
        set(BOX3D_LIB box3d)
    else()
        set(BOX3D_FOUND FALSE)
        set(BOX3D_LIB_INCLUDES "")
        set(BOX3D_LIB "")
    endif()

    #setup include and lib for MuJoCo, a SECOND URDF backend alongside box3d. Also OPTIONAL, and
    #independent of it: absent => MUJOCO_FOUND is FALSE and WITH_MUJOCO_BINDING=0 compiles it out,
    #leaving a build byte-identical to one from before this dependency existed. Box3D remains the
    #default and is unaffected either way — the two are selected per vehicle by "PhysicsEngine".
    #
    #⚠ TWO SEPARATE FLAGS, and conflating them deadlocks the build.
    #  MUJOCO_SOURCE_FOUND — the vendored source is present, so mujoco_wrapper CAN build it.
    #  MUJOCO_FOUND        — a PREBUILT archive is staged, so AirLib can compile and link against it.
    #
    #MuJoCo is not built by build.sh with everything else: it is built out-of-tree by
    #build_thirdparty.sh against UNREAL'S toolchain, for the reasons in cmake-modules/UnrealToolchain.cmake
    #(host glibc >= 2.38 emits __isoc23_* symbols that Unreal's glibc 2.28 sysroot does not have, and
    #libc++ cannot be built without _GNU_SOURCE, so no compiler flag can fix it). Keying MUJOCO_FOUND
    #on the staged archive rather than on the source is what makes "not built yet" and "not vendored"
    #behave identically — both simply leave Box3D as the only backend.
    if(EXISTS "${AIRSIM_ROOT}/external/mujoco/CMakeLists.txt")
        set(MUJOCO_SOURCE_FOUND TRUE)
    else()
        set(MUJOCO_SOURCE_FOUND FALSE)
    endif()

    if(EXISTS "${AIRSIM_ROOT}/AirLib/deps/mujoco/lib/libmujoco.a")
        set(MUJOCO_FOUND TRUE)
        #Headers come from the STAGED copy, not from external/, so the headers and the archive can
        #never disagree about which pin was built.
        set(MUJOCO_LIB_INCLUDES "${AIRSIM_ROOT}/AirLib/deps/mujoco/include")
        set(MUJOCO_LIB "${AIRSIM_ROOT}/AirLib/deps/mujoco/lib/libmujoco.a")
    else()
        set(MUJOCO_FOUND FALSE)
        set(MUJOCO_LIB_INCLUDES "")
        set(MUJOCO_LIB "")
    endif()

    #setup include and lib for CoACD — approximate convex decomposition, shared by BOTH backends
    #rather than owned by either. Same two-flag split as MuJoCo above and for the same reason:
    #COACD_SOURCE_FOUND says the vendored source is here and coacd_wrapper CAN build it,
    #COACD_FOUND says a prebuilt archive is staged so AirLib can compile and link against it.
    #Built out-of-tree by build_thirdparty.sh; absent => WITH_COACD_BINDING=0 and every mesh keeps
    #the single-convex-hull behaviour it has today.
    if(EXISTS "${AIRSIM_ROOT}/external/coacd/CMakeLists.txt")
        set(COACD_SOURCE_FOUND TRUE)
    else()
        set(COACD_SOURCE_FOUND FALSE)
    endif()

    if(EXISTS "${AIRSIM_ROOT}/AirLib/deps/coacd/lib/libcoacd.a")
        set(COACD_FOUND TRUE)
        set(COACD_LIB_INCLUDES "${AIRSIM_ROOT}/AirLib/deps/coacd/include")
        set(COACD_LIB "${AIRSIM_ROOT}/AirLib/deps/coacd/lib/libcoacd.a")
    else()
        set(COACD_FOUND FALSE)
        set(COACD_LIB_INCLUDES "")
        set(COACD_LIB "")
    endif()

    #tinyxml2 is vendored as two source files, compiled straight into AirLib (no separate lib).
    set(TINYXML2_INCLUDES "${AIRSIM_ROOT}/external/tinyxml2")

    #what is our build type debug or release?
    string( TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE)

    IF(UNIX)
        set(RPC_LIB_DEFINES "-D MSGPACK_PP_VARIADICS_MSVC=0")
        set(BUILD_TYPE "linux")
        set(CMAKE_CXX_STANDARD 17)

        if (APPLE)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wstrict-aliasing -D__CLANG__")
        else ()
            set(CMAKE_CXX_FLAGS "\
                -Wall -Wextra \
                -Wnon-virtual-dtor -Woverloaded-virtual \
                -Wno-variadic-macros -Wno-unused-function -Wno-unused \
                -pthread \
                ${RPC_LIB_DEFINES} ${CMAKE_CXX_FLAGS}")

            if (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
                set(CMAKE_CXX_FLAGS "-stdlib=libc++ -Wno-documentation -Wno-unknown-warning-option ${CMAKE_CXX_FLAGS}")
                find_package(LLVM REQUIRED CONFIG)
                set(CXX_EXP_LIB "-L${LLVM_LIBRARY_DIRS} -ferror-limit=10")
            else()
                set(CXX_EXP_LIB "-fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel")
            endif ()
        endif ()

        set(BUILD_PLATFORM "x64")
        set(CMAKE_POSITION_INDEPENDENT_CODE ON)
        if (CMAKE_BUILD_TYPE MATCHES Release)
            set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")
        endif ()

    ELSE()
        #windows cmake build is experimental
        set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_WIN32_WINNT=0x0600 /GS /W4 /wd4100 /wd4505 /wd4820 /wd4464 /wd4514 /wd4710 /wd4571 /Zc:wchar_t /ZI /Zc:inline /fp:precise /D_SCL_SECURE_NO_WARNINGS /D_CRT_SECURE_NO_WARNINGS /D_UNICODE /DUNICODE /WX- /Zc:forScope /Gd /EHsc ")
        set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NXCOMPAT /DYNAMICBASE /INCREMENTAL:NO ")

        if("${BUILD_TYPE}" STREQUAL "debug")
          set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_DEBUG /MDd /RTC1 /Gm /Od ")
        elseif("${BUILD_TYPE}" STREQUAL "release")
          set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MD /O2 /Oi /GL /Gm- /Gy /TP ")
        else()
          message(FATAL_ERROR "Please specify '-D CMAKE_BUILD_TYPE=Debug' or Release on the cmake command line")
        endif()
    ENDIF()

    ## TODO: we are not using Boost any more so below shouldn't be needed
    ## common boost settings to make sure we are all on the same page
    set(Boost_USE_STATIC_LIBS ON)
    set(Boost_USE_MULTITHREADED ON)
    #set(Boost_USE_STATIC_RUNTIME ON)

    ## TODO: probably should set x64 explicitly
    ## strip x64 from /machine:x64 from CMAKE_STATIC_LINKER_FLAGS and set in BUILD_PLATFORM
    if(NOT "${CMAKE_STATIC_LINKER_FLAGS}" STREQUAL "")
      string(SUBSTRING ${CMAKE_STATIC_LINKER_FLAGS} 9 -1 "BUILD_PLATFORM")
    endif()

endmacro(CommonSetup)
