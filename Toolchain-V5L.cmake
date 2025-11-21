# Cross-compilation toolchain for ARM64 (aarch64) with ROS, CUDA, and TensorRT
cmake_minimum_required(VERSION 3.10)

# Target system configuration
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu)

# Paths - centralized for easy maintenance
set(SYSROOT "$ENV{HOME}/nvidia_ws/target_fs") #you can comment if you want but like why?
set(DRIVEWORKS_ROOT "${SYSROOT}/usr/local/driveworks")
set(CUDA_ROOT "${SYSROOT}/usr/local/cuda-11.4")
set(TENSORRT_ROOT "${SYSROOT}/usr")

# Cross-compilation settings
set(CMAKE_SYSROOT ${SYSROOT})
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})

# Compilers
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Search modes - restrict to target environment only
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# ROS configuration
set(CMAKE_PREFIX_PATH "${SYSROOT}/opt/ros/noetic")
set(CMAKE_LIBRARY_PATH "${SYSROOT}/opt/ros/noetic/lib" CACHE STRING "ROS Libraries Path")
set(CMAKE_INCLUDE_PATH "${SYSROOT}/opt/ros/noetic/include" CACHE STRING "ROS Include Path")
set(CMAKE_SYSTEM_IGNORE_PATH "/opt/ros/noetic/lib" "/opt/ros/noetic/include")

# Library search paths - build time
set(LIB_PATHS
    "${SYSROOT}/usr/lib/aarch64-linux-gnu"
    "${SYSROOT}/lib/aarch64-linux-gnu"
    "${DRIVEWORKS_ROOT}/targets/aarch64-Linux/lib"
    "${CUDA_ROOT}/targets/aarch64-linux/lib"
    "${TENSORRT_ROOT}/lib/aarch64-linux-gnu"
)

# Make sure DriveWorks lib directory exists
if(NOT EXISTS "${DRIVEWORKS_ROOT}/targets/aarch64-Linux/lib")
    message(WARNING "DriveWorks lib directory not found: ${DRIVEWORKS_ROOT}/targets/aarch64-Linux/lib")
else()
    message(STATUS "DriveWorks lib directory found: ${DRIVEWORKS_ROOT}/targets/aarch64-Linux/lib")
endif()

# Runtime library paths (rpath) - these are the paths on the target system
set(RUNTIME_PATHS
    "/usr/lib/aarch64-linux-gnu"
    "/lib/aarch64-linux-gnu"
    "/usr/local/driveworks/targets/aarch64-Linux/lib"
    "/usr/local/cuda-11.4/targets/aarch64-linux/lib"
    "/usr/lib/aarch64-linux-gnu"
)

# Build linker search paths
string(REPLACE ";" " -L" LIB_SEARCH_FLAGS "${LIB_PATHS}")
set(LIB_SEARCH_FLAGS "-L${LIB_SEARCH_FLAGS}")

# Runtime linker paths
string(REPLACE ";" " -Wl,-rpath," RPATH_FLAGS "${RUNTIME_PATHS}")
set(RPATH_FLAGS "-Wl,-rpath,${RPATH_FLAGS}")

# Compiler flags - added C++14 standard and other necessary flags
set(CMAKE_C_FLAGS "-fPIC --sysroot=${CMAKE_SYSROOT} -g" CACHE INTERNAL "" FORCE)
set(CMAKE_CXX_FLAGS "-fPIC --sysroot=${CMAKE_SYSROOT} -std=c++14 -g -fpermissive" CACHE INTERNAL "" FORCE)

# Linker flags - unified for all link types
set(COMMON_LINK_FLAGS "--sysroot=${CMAKE_SYSROOT} ${LIB_SEARCH_FLAGS} ${RPATH_FLAGS} -Wl,--allow-shlib-undefined -Wl,--unresolved-symbols=ignore-in-shared-libs")
set(CMAKE_EXE_LINKER_FLAGS "${COMMON_LINK_FLAGS}" CACHE STRING "executable linker flags")
set(CMAKE_SHARED_LINKER_FLAGS "${COMMON_LINK_FLAGS}" CACHE STRING "shared linker flags")
set(CMAKE_MODULE_LINKER_FLAGS "${COMMON_LINK_FLAGS}" CACHE STRING "module linker flags")

# Include directories - system includes
include_directories(BEFORE SYSTEM 
    "${SYSROOT}/usr/include/aarch64-linux-gnu"
    "${DRIVEWORKS_ROOT}/include"
    "${SYSROOT}/usr/include/nvmedia_6x"
    "${CUDA_ROOT}/targets/aarch64-linux/include"
    "${TENSORRT_ROOT}/include/aarch64-linux-gnu"
)

# Add library directories for find_library commands
link_directories(${LIB_PATHS})

set(Boost_NO_SYSTEM_PATHS ON)
set(Boost_NO_BOOST_CMAKE ON)
set(BOOST_ROOT "${SYSROOT}/usr")
set(BOOST_INCLUDEDIR "${SYSROOT}/usr/include")
set(BOOST_LIBRARYDIR "${SYSROOT}/usr/lib/aarch64-linux-gnu")


# Environment variables for TensorRT
set(ENV{TENSORRT_ROOT} ${TENSORRT_ROOT})

message(STATUS "Cross-compilation toolchain loaded:")
message(STATUS "  Target: ${CMAKE_SYSTEM_PROCESSOR}-${CMAKE_SYSTEM_NAME}")
message(STATUS "  Sysroot: ${CMAKE_SYSROOT}")
message(STATUS "  Compilers: ${CMAKE_C_COMPILER}, ${CMAKE_CXX_COMPILER}")
message(STATUS "  Library paths: ${LIB_PATHS}")
message(STATUS "  TensorRT root: ${TENSORRT_ROOT}")

#list(PREPEND CMAKE_PREFIX_PATH 
#    "/opt/ros/noetic" 
#)
