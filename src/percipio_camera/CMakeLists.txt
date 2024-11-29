cmake_minimum_required(VERSION 3.5)
project(percipio_camera)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -fPIC -g3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fPIC -g3")
set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
  ament_cmake
  ament_index_cpp
  Eigen3
  builtin_interfaces
  cv_bridge
  camera_info_manager
  image_transport
  image_publisher
  OpenCV
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_msgs
  tf2_ros
  tf2_sensor_msgs
  Threads
  diagnostic_updater
  diagnostic_msgs
)

foreach (dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach ()

#find_package(ament_cmake REQUIRED)
#find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
#find_package(sensor_msgs REQUIRED)

find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
#find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)

set(SOURCE_FILES
  src/percipio_camera_node_driver.cpp
  src/percipio_camera_node.cpp
  src/percipio_device.cpp
  src/percipio_video_mode.cpp
  src/TYThread.cpp
  src/DepthInpainter.cpp
  src/ImageSpeckleFilter.cpp
  src/MatViewer.cpp
  src/huffman.cpp
)

set(SOURCE_INCLUDES_DIR
  include
)

set(SDK_INCLUDE_DIRS
  camport3/include
  ${OpenCV_INCLUDED_DIRS}
)

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

set(SDK_LIBRARIES ${CMAKE_CURRENT_SOURCE_DIR}/camport3/lib/linux/lib_${ARCH}/libtycam.so)

macro(add_percipio_ros_executable TARGET SOURCE)
  add_executable(${TARGET} ${SOURCE} src/DepthInpainter.cpp  src/ImageSpeckleFilter.cpp src/MatViewer.cpp src/TYThread.cpp)
  target_include_directories(${TARGET} PUBLIC ${SDK_INCLUDE_DIRS} ${SOURCE_INCLUDES_DIR})
  target_link_libraries(${TARGET} ${SDK_LIBRARIES} ${PROJECT_NAME})
  ament_target_dependencies(${TARGET} ${dependencies})
endmacro()

add_percipio_ros_executable(list_devices src/list_devices.cpp)

find_package(PCL REQUIRED)
if(NOT PCL_FOUND)
    message(FATAL_ERROR "PCL library not found")
else()
    find_package(OpenCV REQUIRED)
    if (NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV library not found")
    else()
        message(STATUS "OpenCV version: ${OpenCV_VERSION}")
        message(STATUS "PLC version: ${PLC_VERSION}")
        include_directories(${PCL_INCLUDE_DIRS})
        include_directories(${OpenCV_INCLUDE_DIRS})
        include_directories(${SOURCE_INCLUDES_DIR})

        add_library(${PROJECT_NAME} SHARED
            ${SOURCE_FILES}
        )

        ament_target_dependencies(${PROJECT_NAME} ${dependencies})
        target_include_directories(${PROJECT_NAME} PUBLIC ${SDK_INCLUDE_DIRS} ${SOURCE_INCLUDES_DIR})
        target_link_libraries(${PROJECT_NAME} ${SDK_LIBRARIES})
        
        rclcpp_components_register_node(${PROJECT_NAME}
            PLUGIN "percipio_camera::PercipioCameraNodeDriver"
            EXECUTABLE percipio_camera_node
        )

        install(TARGETS ${PROJECT_NAME}
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
        )
        
        file(GLOB lib_files ${CMAKE_CURRENT_SOURCE_DIR}/camport3/lib/linux/lib_${ARCH}/*) 

        install(FILES ${lib_files} DESTINATION lib)
        install(DIRECTORY include/ DESTINATION include)
        install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
        install(TARGETS ${PROJECT_NAME}
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
        )
            
        ament_export_libraries(${PROJECT_NAME})
    endif()
endif()

install(TARGETS list_devices
  DESTINATION lib/${PROJECT_NAME}/)

ament_package()
