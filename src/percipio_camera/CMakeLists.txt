cmake_minimum_required(VERSION 3.5)
project(percipio_camera)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_STANDARD 11)

# Default to Release if no build type is specified, but allow -DCMAKE_BUILD_TYPE=Debug to work.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fPIC -g3")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fPIC -g3")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
  ament_cmake
  ament_index_cpp
  builtin_interfaces
  camera_info_manager
  cv_bridge
  diagnostic_msgs
  diagnostic_updater
  Eigen3
  geometry_msgs
  image_publisher
  image_transport
  lifecycle_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  tf2_sensor_msgs
)

foreach (dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach ()

find_package(Threads REQUIRED)

if(cv_bridge_VERSION VERSION_GREATER_EQUAL "3.3.0")
    add_definitions(-DUSE_CV_BRIDGE_HPP=1)
else()
    add_definitions(-DUSE_CV_BRIDGE_HPP=0)
endif()

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "i686")
  set(ARCH i686)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(ARCH x64)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(ARCH Aarch64)
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "armv7")
  set(ARCH armv7hf)
else()
  message(STATUS "host processor architecture is unknown")
endif()

find_package(TYCam REQUIRED PATHS ${CMAKE_CURRENT_SOURCE_DIR}/camport4 NO_DEFAULT_PATH)
include_directories(${TYCam_INCLUDE_DIRS})

find_package(PercipioFirmwareUpdate REQUIRED PATHS ${CMAKE_CURRENT_SOURCE_DIR}/camport4 NO_DEFAULT_PATH)
include_directories(${PercipioFirmwareUpdate_INCLUDE_DIRS})

set(SOURCE_FILES
  src/percipio_camera_node_driver.cpp
  src/percipio_camera_node.cpp
  src/percipio_device.cpp
  src/percipio_log_server.cpp
  src/gige_2_0.cpp
  src/gige_2_1.cpp
  src/percipio_xml.cpp
  src/percipio_video_mode.cpp
  src/TYThread.cpp
  src/percipio_depth_algorithm.cpp
  src/huffman.cpp
)

file(GLOB_RECURSE LEGACY_SOURCES
    "src/huffman.cpp"
    "src/percipio_xml.cpp"
)

set_source_files_properties(${LEGACY_SOURCES}
    PROPERTIES
        COMPILE_FLAGS "-Wno-implicit-fallthrough -Wno-format -Wno-parentheses"
)

set(SOURCE_INCLUDES_DIR
  include
)

set(SDK_INCLUDE_DIRS
  ${TYCam_INCLUDE_DIRS}
)


add_executable(firmware_update src/percipio_firmware_update.cpp)
target_include_directories(firmware_update SYSTEM PUBLIC ${PercipioFirmwareUpdate_INCLUDE_DIRS})
target_link_libraries(firmware_update ${PercipioFirmwareUpdate_LIB})



macro(add_percipio_ros_executable TARGET SOURCE)
  add_executable(${TARGET} ${SOURCE})
  #target_include_directories(${TARGET} PUBLIC ${SDK_INCLUDE_DIRS} ${SOURCE_INCLUDES_DIR})
  ament_target_dependencies(${TARGET} ${dependencies})
  target_include_directories(${TARGET} SYSTEM PUBLIC ${SDK_INCLUDE_DIRS})
  target_include_directories(${TARGET} PUBLIC ${SOURCE_INCLUDES_DIR})
  target_link_libraries(${TARGET} ${TYCam_LIBS} ${PROJECT_NAME} Threads::Threads)
endmacro()

add_percipio_ros_executable(list_devices src/list_devices.cpp)
add_percipio_ros_executable(network_ip_config src/network_ip_config.cpp)

message(STATUS "PLC version: ${PLC_VERSION}")
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${SOURCE_INCLUDES_DIR})

add_library(${PROJECT_NAME} SHARED
    ${SOURCE_FILES}
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
#target_include_directories(${PROJECT_NAME} PUBLIC ${SDK_INCLUDE_DIRS} ${SOURCE_INCLUDES_DIR})
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${SDK_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${SOURCE_INCLUDES_DIR})
target_link_libraries(${PROJECT_NAME} ${TYCam_LIBS} ${rclcpp_LIBRARIES})

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "percipio_camera::PercipioCameraNodeDriver"
    EXECUTABLE percipio_camera_node
)

ament_export_libraries(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

file(GLOB lib_files ${CMAKE_CURRENT_SOURCE_DIR}/camport4/lib/linux/lib_${ARCH}/*) 
install(FILES ${lib_files} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS list_devices
  DESTINATION lib/${PROJECT_NAME}/)

install(TARGETS network_ip_config
  DESTINATION lib/${PROJECT_NAME}/)

install(TARGETS firmware_update
  DESTINATION lib/${PROJECT_NAME}/)
  
ament_package()
