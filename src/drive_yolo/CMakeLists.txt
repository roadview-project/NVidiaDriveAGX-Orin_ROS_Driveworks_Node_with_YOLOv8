cmake_minimum_required(VERSION 3.0.2)
project(drive_yolo)

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

# Force Boost to use sysroot paths
set(CMAKE_FIND_LIBRARY_PREFIXES "lib")
set(CMAKE_FIND_LIBRARY_SUFFIXES ".so" ".a")

# Explicitly set Boost library paths
set(BOOST_ROOT ${CMAKE_SYSROOT}/usr)
set(BOOST_LIBRARYDIR ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu)

# Find Boost
find_package(Boost REQUIRED COMPONENTS 
    system 
    thread 
    chrono 
    filesystem
    date_time
    regex
)

# Override the imported targets to use sysroot paths
if(Boost_FOUND)
    message(STATUS "Boost found, fixing library paths...")
    foreach(component system thread chrono filesystem date_time regex)
        if(TARGET Boost::${component})
            get_target_property(boost_lib Boost::${component} LOCATION)
            if(boost_lib)
                # Replace /usr/lib with ${CMAKE_SYSROOT}/usr/lib
                string(REPLACE "/usr/lib" "${CMAKE_SYSROOT}/usr/lib" boost_lib_fixed "${boost_lib}")
                set_target_properties(Boost::${component} PROPERTIES
                    IMPORTED_LOCATION "${boost_lib_fixed}"
                )
                message(STATUS "  Fixed Boost::${component} -> ${boost_lib_fixed}")
            endif()
        endif()
    endforeach()
endif()


find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs 
    sensor_msgs         # Make sure this is found
    geometry_msgs       # NEW: For Point messages in fusion
    message_generation
    message_filters     # For visualizer synchronization
    tf2_ros          
    tf2_geometry_msgs 
    cv_bridge

)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d)

add_message_files(
    FILES
    Detection.msg
    Detections.msg
    DetectionWithDistance.msg      # NEW: Fusion message
    DetectionsWithDistance.msg     # NEW: Fusion message
    TrackedObject.msg
    TrackedObjects.msg
    TrackedObject3D.msg            # NEW: 3D tracking message
    TrackedObjects3D.msg           # NEW: 3D tracking message
    ProjectedPoint.msg             # NEW: Point cloud projection message
    ProjectedPoints.msg            # NEW: Point cloud projection message array
)
#catkin_python_setup()

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs         # CRITICAL: This was missing
    geometry_msgs       # NEW: For fusion messages
)

# DriveWorks configuration - keep your existing setup
#set(DRIVEWORKS_ROOT "$ENV{HOME}/drive-ros/driveworks/driveworks-5.10")
set(DRIVEWORKS_LIB_DIR "${DRIVEWORKS_ROOT}/targets/aarch64-Linux/lib")
set(DRIVEWORKS_INCLUDE_DIR "${DRIVEWORKS_ROOT}/include")

# Check if DriveWorks directory exists
if(EXISTS "${DRIVEWORKS_ROOT}")
    message(STATUS "DriveWorks root found: ${DRIVEWORKS_ROOT}")
else()
    message(WARNING "DriveWorks root not found: ${DRIVEWORKS_ROOT}")
endif()

# Find DriveWorks libraries with explicit full paths
set(DRIVEWORKS_BASE_LIB "${DRIVEWORKS_LIB_DIR}/libdw_base.so")
set(DRIVEWORKS_SENSORS_LIB "${DRIVEWORKS_LIB_DIR}/libdw_sensors.so")  
set(DRIVEWORKS_SHARED_LIB "${DRIVEWORKS_LIB_DIR}/libdwshared.so")

# Check if the library files actually exist
if(EXISTS "${DRIVEWORKS_BASE_LIB}")
    message(STATUS "Found DriveWorks base: ${DRIVEWORKS_BASE_LIB}")
    set(DW_BASE_FOUND TRUE)
else()
    message(WARNING "DriveWorks base not found: ${DRIVEWORKS_BASE_LIB}")
    set(DW_BASE_FOUND FALSE)
endif()

if(EXISTS "${DRIVEWORKS_SENSORS_LIB}")
    message(STATUS "Found DriveWorks sensors: ${DRIVEWORKS_SENSORS_LIB}")
    set(DW_SENSORS_FOUND TRUE)
else()
    message(WARNING "DriveWorks sensors not found: ${DRIVEWORKS_SENSORS_LIB}")
    set(DW_SENSORS_FOUND FALSE)
endif()

if(EXISTS "${DRIVEWORKS_SHARED_LIB}")
    message(STATUS "Found DriveWorks shared: ${DRIVEWORKS_SHARED_LIB}")
    set(DW_SHARED_FOUND TRUE)
else()
    message(WARNING "DriveWorks shared not found: ${DRIVEWORKS_SHARED_LIB}")
    set(DW_SHARED_FOUND FALSE)
endif()

# Set DRIVEWORKS_FOUND based on all required libraries
if(DW_BASE_FOUND AND DW_SENSORS_FOUND AND DW_SHARED_FOUND)
    set(DRIVEWORKS_FOUND TRUE)
    message(STATUS "All DriveWorks libraries found!")
else()
    set(DRIVEWORKS_FOUND FALSE)
    message(WARNING "Some DriveWorks libraries missing - compiling without DriveWorks")
endif()

# TensorRT configuration for cross-compilation - keep your existing setup
#set(TENSORRT_ROOT "${CMAKE_FIND_ROOT_PATH}/usr")
set(TENSORRT_INCLUDE_DIR "${TENSORRT_ROOT}/include/aarch64-linux-gnu")
set(TENSORRT_LIB_DIR "${TENSORRT_ROOT}/lib/aarch64-linux-gnu")

# Check if TensorRT headers exist
if(EXISTS "${TENSORRT_INCLUDE_DIR}/NvInfer.h")
    message(STATUS "Found TensorRT headers at: ${TENSORRT_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "TensorRT headers not found at: ${TENSORRT_INCLUDE_DIR}")
endif()

# Find TensorRT libraries with explicit paths
find_library(TENSORRT_LIB nvinfer 
    PATHS ${TENSORRT_LIB_DIR}
    NO_DEFAULT_PATH
    REQUIRED
)

find_library(NVONNX_LIB nvonnxparser 
    PATHS ${TENSORRT_LIB_DIR}
    NO_DEFAULT_PATH
    REQUIRED
)

find_library(CUDART_LIB cudart
    PATHS ${CMAKE_FIND_ROOT_PATH}/usr/local/cuda-11.4/targets/aarch64-linux/lib
    NO_DEFAULT_PATH
    REQUIRED
)

# Print found libraries for debugging
message(STATUS "Found TensorRT library: ${TENSORRT_LIB}")
message(STATUS "Found NVONNX library: ${NVONNX_LIB}")
message(STATUS "Found CUDART library: ${CUDART_LIB}")

# Catkin package configuration - updated dependencies
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs message_runtime message_filters tf2_ros tf2_geometry_msgs cv_bridge
    DEPENDS OpenCV
)

catkin_install_python(PROGRAMS
    scripts/tracking_3d_node.py
    scripts/fused_data2ros2.py
    scripts/detection_visualizer_node.py
    scripts/osi2ros_lidar.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Include directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${TENSORRT_INCLUDE_DIR}
    ${CMAKE_FIND_ROOT_PATH}/usr/local/cuda-11.4/targets/aarch64-linux/include
)

# Add VIBRANTE version definitions for DRIVE OS 6.0.6
# This ensures DriveWorks headers use the correct API version
add_compile_definitions(
    VIBRANTE_PDK_DECIMAL=6000600
    VIBRANTE_PDK_VERSION=6.0.6.0
    VIBRANTE
)



# Create the main YOLO executable
add_executable(${PROJECT_NAME}_node 
    src/yolo_node.cpp 
    src/yolo_infer.cpp
    src/driveworks_camera.cpp
)

add_executable(${PROJECT_NAME}_tracking_3d_node
    src/tracking_3d_node.cpp
    src/object_tracker_3d.cpp
)

add_executable(${PROJECT_NAME}_speed_monitor_node
    src/speed_monitor_node.cpp
)

# Create image publisher for testing
add_executable(${PROJECT_NAME}_image_publisher
    src/image_publisher.cpp
)

# Create visualizer executable
add_executable(${PROJECT_NAME}_visualizer
    src/yolo_visualizer.cpp
)

# NEW: Create LiDAR fusion executable
add_executable(${PROJECT_NAME}_lidar_fusion_node
    src/lidar_camera_fusion_node.cpp
    src/lidar_camera_fusion.cpp
)

# Create camera info publisher for RViz pointcloud overlay
add_executable(${PROJECT_NAME}_camera_info_publisher
    src/camera_info_publisher.cpp
)

# Create synchronization diagnostics node
add_executable(${PROJECT_NAME}_sync_diagnostics
    src/sync_diagnostics_node.cpp
)

# Create pointcloud delay node for testing/visualization
add_executable(${PROJECT_NAME}_pointcloud_delay
    src/pointcloud_delay_node.cpp
)

# NEW: Modular fusion pipeline nodes
add_executable(${PROJECT_NAME}_pointcloud_filter
    src/pointcloud_filter_node.cpp
)

add_executable(${PROJECT_NAME}_pointcloud_projector
    src/pointcloud_projector_node.cpp
)

# NEW: UDP vehicle TF publisher
add_executable(${PROJECT_NAME}_udp_vehicle_tf
    src/udp_vehicle_tf_publisher.cpp
)

# NEW: Static object detector
add_executable(${PROJECT_NAME}_static_object_detector
    src/static_object_detector_node.cpp
)

set_target_properties(${PROJECT_NAME}_tracking_3d_node PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

set_target_properties(${PROJECT_NAME}_speed_monitor_node PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

# Set target properties for all executables
set_target_properties(${PROJECT_NAME}_node PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

set_target_properties(${PROJECT_NAME}_image_publisher PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

set_target_properties(${PROJECT_NAME}_visualizer PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

set_target_properties(${PROJECT_NAME}_lidar_fusion_node PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

set_target_properties(${PROJECT_NAME}_camera_info_publisher PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

set_target_properties(${PROJECT_NAME}_sync_diagnostics PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

set_target_properties(${PROJECT_NAME}_pointcloud_delay PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

# NEW: Set properties for new nodes
set_target_properties(${PROJECT_NAME}_pointcloud_filter PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

set_target_properties(${PROJECT_NAME}_pointcloud_projector PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

set_target_properties(${PROJECT_NAME}_udp_vehicle_tf PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

set_target_properties(${PROJECT_NAME}_static_object_detector PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED ON
)

# Link libraries to the main YOLO executable
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    ${TENSORRT_LIB}
    ${NVONNX_LIB}
    ${CUDART_LIB}
    pthread
    dl
    rt
    -Wl,--no-as-needed
)

# Add DriveWorks libraries if found
if(DRIVEWORKS_FOUND)
    target_link_libraries(${PROJECT_NAME}_node
        ${DRIVEWORKS_BASE_LIB}
        ${DRIVEWORKS_SENSORS_LIB}
        ${DRIVEWORKS_SHARED_LIB}
    )
    target_compile_definitions(${PROJECT_NAME}_node PRIVATE USE_DRIVEWORKS=1)
    message(STATUS "Compiling with DriveWorks support")
else()
    message(STATUS "Compiling without DriveWorks support")
endif()

# Link libraries to the image publisher
target_link_libraries(${PROJECT_NAME}_image_publisher
    ${catkin_LIBRARIES}
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
)

# Link libraries to the visualizer
target_link_libraries(${PROJECT_NAME}_visualizer
    ${catkin_LIBRARIES}
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
)

target_link_libraries(${PROJECT_NAME}_lidar_fusion_node
    ${catkin_LIBRARIES}
    opencv_core
    opencv_imgproc
    opencv_calib3d
    pthread
    dl
    rt
    -Wl,--no-as-needed
)

target_link_libraries(${PROJECT_NAME}_tracking_3d_node
    ${catkin_LIBRARIES}
    opencv_core
    opencv_imgproc
    opencv_video        # For Kalman filter
    pthread
    dl
    rt
    -Wl,--no-as-needed
)

# Link libraries to the speed monitor node
target_link_libraries(${PROJECT_NAME}_speed_monitor_node
    ${catkin_LIBRARIES}
    pthread
    dl
    rt
    -Wl,--no-as-needed
)

# Link libraries to the camera info publisher
target_link_libraries(${PROJECT_NAME}_camera_info_publisher
    ${catkin_LIBRARIES}
)

# Link libraries to the sync diagnostics node
target_link_libraries(${PROJECT_NAME}_sync_diagnostics
    ${catkin_LIBRARIES}
)

# Link libraries to the pointcloud delay node
target_link_libraries(${PROJECT_NAME}_pointcloud_delay
    ${catkin_LIBRARIES}
)

# NEW: Link libraries for new nodes
target_link_libraries(${PROJECT_NAME}_pointcloud_filter
    ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_pointcloud_projector
    ${catkin_LIBRARIES}
    opencv_core
    opencv_imgproc
    opencv_calib3d
    pthread
    dl
    rt
    -Wl,--no-as-needed
)

target_link_libraries(${PROJECT_NAME}_udp_vehicle_tf
    ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_static_object_detector
    ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_tracking_3d_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
)

add_dependencies(${PROJECT_NAME}_speed_monitor_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
)

add_dependencies(${PROJECT_NAME}_camera_info_publisher
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}_sync_diagnostics
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
)

add_dependencies(${PROJECT_NAME}_pointcloud_delay
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

# NEW: Add dependencies for new nodes
add_dependencies(${PROJECT_NAME}_pointcloud_filter
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}_pointcloud_projector
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
)

add_dependencies(${PROJECT_NAME}_udp_vehicle_tf
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}_static_object_detector
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
)

# Add dependencies for message generation
add_dependencies(${PROJECT_NAME}_node 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
)

add_dependencies(${PROJECT_NAME}_image_publisher
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

add_dependencies(${PROJECT_NAME}_visualizer
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
)

# NEW: Add dependencies for fusion node
add_dependencies(${PROJECT_NAME}_lidar_fusion_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
)

# Install rules
install(TARGETS
    ${PROJECT_NAME}_node
    ${PROJECT_NAME}_image_publisher
    ${PROJECT_NAME}_visualizer
    ${PROJECT_NAME}_lidar_fusion_node
    ${PROJECT_NAME}_tracking_3d_node        # NEW
    ${PROJECT_NAME}_speed_monitor_node      # NEW
    ${PROJECT_NAME}_camera_info_publisher   # Camera info for RViz overlay
    ${PROJECT_NAME}_sync_diagnostics        # Sync diagnostics for timing debug
    ${PROJECT_NAME}_pointcloud_delay        # Pointcloud delay for testing/demo
    ${PROJECT_NAME}_pointcloud_filter       # NEW: Modular fusion - filter
    ${PROJECT_NAME}_pointcloud_projector    # NEW: Modular fusion - projector
    ${PROJECT_NAME}_udp_vehicle_tf          # NEW: UDP vehicle TF publisher
    ${PROJECT_NAME}_static_object_detector  # NEW: Static object detector
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)

install(FILES requirements.txt
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# Optional: Create a library version as well
add_library(${PROJECT_NAME} src/yolo_infer.cpp)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    opencv_core
    opencv_imgproc
    ${TENSORRT_LIB}
    ${NVONNX_LIB}
    ${CUDART_LIB}
)



add_custom_target(check_python_deps
    COMMAND ${CMAKE_COMMAND} -E echo "Checking Python dependencies..."
    COMMAND python3 -c "import motpy\; import numpy\; import scipy\; import sklearn"
    COMMENT "Checking Python dependencies for 3D tracking"
)

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ${TENSORRT_INCLUDE_DIR}
)

add_dependencies(${PROJECT_NAME}_node check_python_deps)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY models/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
    FILES_MATCHING 
    PATTERN "*.engine"
    PATTERN "*.onnx"
    PATTERN "*.pt"
)
