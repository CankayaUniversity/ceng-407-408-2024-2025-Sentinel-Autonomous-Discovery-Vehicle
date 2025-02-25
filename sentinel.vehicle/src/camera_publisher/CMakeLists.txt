cmake_minimum_required(VERSION 3.8)


project(camera_publisher)

set(EXECUTABLE_NAME camera)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

file(GLOB SRC_FILES "${CMAKE_SOURCE_DIR}/src/*.cpp")
file(GLOB HEADER_FILES "${CMAKE_SOURCE_DIR}/include/*.hpp" "${CMAKE_SOURCE_DIR}/include/*/*.hpp")

add_executable(${EXECUTABLE_NAME}
  ${SRC_FILES}
  ${HEADER_FILES}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig)

pkg_check_modules(LIBCAMERA IMPORTED_TARGET libcamera)
pkg_check_modules(LIBEVENT IMPORTED_TARGET libevent_pthreads)

if(LIBCAMERA_FOUND)
    message(STATUS "libcamera library found:")
    message(STATUS "    version: ${LIBCAMERA_VERSION}")
    message(STATUS "    libraries: ${LIBCAMERA_LINK_LIBRARIES}")
    message(STATUS "    include path: ${LIBCAMERA_INCLUDE_DIRS}")

    target_link_libraries(${EXECUTABLE_NAME} PkgConfig::LIBCAMERA)
    target_compile_definitions(${EXECUTABLE_NAME} PRIVATE LIBCAMERA)
else()
    message(WARNING "libcamera not found, skipping linking.")
endif()

if(LIBEVENT_FOUND)
    message(STATUS "libevent_pthreads library found:")
    message(STATUS "    version: ${LIBEVENT_VERSION}")
    message(STATUS "    libraries: ${LIBEVENT_LINK_LIBRARIES}")
    message(STATUS "    include path: ${LIBEVENT_INCLUDE_DIRS}")

    target_link_libraries(${EXECUTABLE_NAME} PkgConfig::LIBEVENT)
else()
    message(WARNING "libevent_pthreads not found, skipping linking.")
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EXECUTABLE_NAME} PUBLIC include)
target_link_libraries(${EXECUTABLE_NAME} ${OpenCV_LIBS})

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


install(TARGETS
 ${EXECUTABLE_NAME}
  DESTINATION lib/${CMAKE_PROJECT_NAME})

ament_target_dependencies(${EXECUTABLE_NAME} rclcpp)
ament_package()
