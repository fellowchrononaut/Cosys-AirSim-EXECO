cmake_minimum_required(VERSION 3.12.0)
project(AirSim)

# Focused, dependency-free AirLib tests register with CTest from their owning subdirectories.
enable_testing()

add_subdirectory("rpclib_wrapper")
add_subdirectory("box3d_wrapper")
add_subdirectory("mujoco_wrapper")
add_subdirectory("coacd_wrapper")
add_subdirectory("AirLib")
add_subdirectory("MavLinkCom")
add_subdirectory("AirLibUnitTests")
add_subdirectory("HelloDrone")
add_subdirectory("HelloSpawnedDrones")
add_subdirectory("HelloCar")
add_subdirectory("DroneShell")
add_subdirectory("DroneServer")

