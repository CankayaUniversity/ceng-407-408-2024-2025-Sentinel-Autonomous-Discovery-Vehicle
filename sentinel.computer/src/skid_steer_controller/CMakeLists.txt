cmake_minimum_required(VERSION 3.8)
project(skid_steer_controller)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_toolbox
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
add_compile_definitions(RCPPUTILS_VERSION_MAJOR=${rcpputils_VERSION_MAJOR})
add_compile_definitions(RCPPUTILS_VERSION_MINOR=${rcpputils_VERSION_MINOR})
add_compile_options(-Wno-shadow)




get_target_property(TB_INCLUDE_DIRS control_toolbox::rate_limiter_parameters INTERFACE_INCLUDE_DIRECTORIES)
generate_parameter_library(skid_steer_controller_parameters
  src/skid_steer_controller_parameter.yaml
  ${TB_INCLUDE_DIRS}/control_toolbox/custom_validators.hpp
)
file(GLOB SRC_FILES "${CMAKE_SOURCE_DIR}/src/*.cpp")
file(GLOB HEADER_FILES "${CMAKE_SOURCE_DIR}/include/*.hpp" "${CMAKE_SOURCE_DIR}/include/*/*.hpp")

add_library(skid_steer_controller SHARED
  ${SRC_FILES}
  ${HEADER_FILES}
)



target_link_libraries(skid_steer_controller
  PUBLIC
    skid_steer_controller_parameters
    control_toolbox::rate_limiter_parameters)
include_directories(skid_steer_controller PUBLIC include)
target_include_directories(skid_steer_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(skid_steer_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(skid_steer_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
pluginlib_export_plugin_description_file(controller_interface skid_steer_plugin.xml)




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include/skid_steer_controller
)

install(TARGETS skid_steer_controller skid_steer_controller_parameters
  EXPORT export_skid_steer_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
ament_export_targets(export_skid_steer_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
