cmake_minimum_required(VERSION 3.8)
project(sentinel_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ament_index_cpp
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
)

# find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
# if(NOT WIRINGPI_LIBRARIES)
#   message(FATAL_ERROR "WiringPi library not found. Please install WiringPi.")
# endif()

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


file(GLOB SRC_FILES "${CMAKE_SOURCE_DIR}/src/*.cpp")
file(GLOB HEADER_FILES "${CMAKE_SOURCE_DIR}/include/*.hpp" "${CMAKE_SOURCE_DIR}/include/*/*.hpp")

add_library(sentinel_hardware_interface SHARED
  ${SRC_FILES}
  ${HEADER_FILES}
)
# target_link_libraries(${PROJECT_NAME} PUBLIC
#   ${WIRINGPI_LIBRARIES}
# )


target_link_libraries(${PROJECT_NAME} PUBLIC serial)

target_include_directories(sentinel_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(sentinel_hardware_interface PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

include_directories(sentinel_hardware_interface PUBLIC include)
target_compile_features(sentinel_hardware_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_definitions(${PROJECT_NAME} PRIVATE "SENTINEL_HARDWARE_INTERFACE_BUILDING_DLL")
pluginlib_export_plugin_description_file(hardware_interface sentinel_hardware_interface.xml)


# install(TARGETS sentinel_hardware_interface
#   DESTINATION lib/${PROJECT_NAME})

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
  DESTINATION include/sentinel_hardware_interface
)

install(TARGETS sentinel_hardware_interface
  EXPORT export_sentinel_hardware_interfece
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



ament_export_targets(export_sentinel_hardware_interfece HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
