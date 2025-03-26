cmake_minimum_required(VERSION 3.10)
project(skid_steering_plugin)

find_package(ament_cmake REQUIRED)
if(${ament_cmake_FOUND})
  message("Building ${PROJECT_NAME} as an `ament_cmake` project.")
endif()

find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

gz_find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

gz_find_package(gz-rendering8 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

gz_find_package(gz-physics7 REQUIRED)
set(GZ_PHYSICS_VER ${gz-physics7_VERSION_MAJOR})

message(STATUS "Compiling against Gazebo Harmonic")

pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(skid_steering_plugin SHARED src/skid_steering_plugin.cc)

# Include Gazebo headers
target_include_directories(skid_steering_plugin PRIVATE include)

# Link against Gazebo 
target_link_libraries(skid_steering_plugin  
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
  gz-physics${GZ_PHYSICS_VER}::gz-physics${GZ_PHYSICS_VER}
)

ament_target_dependencies(skid_steering_plugin rclcpp geometry_msgs tf2_ros nav_msgs)

ament_export_dependencies(gz-sim${GZ_SIM_VER} gz-rendering${GZ_RENDERING_VER} gz-common${GZ_COMMON_VER} gz-physics${GZ_PHYSICS_VER})
ament_export_libraries(skid_steering_plugin)

install(DIRECTORY include/ 
  DESTINATION include
)

install(
  TARGETS skid_steering_plugin 
  DESTINATION lib/
)

ament_package()
