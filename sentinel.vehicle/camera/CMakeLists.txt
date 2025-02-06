cmake_minimum_required(VERSION 3.8)
project(sentinel.vehicle.camera)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


file(GLOB SRC_FILES "${CMAKE_SOURCE_DIR}/src/*.cpp")
file(GLOB HEADER_FILES "${CMAKE_SOURCE_DIR}/include/*.hpp")

message(${SRC_FILES})
message(${HEADER_FILES})

add_executable(CameraPublisher
  ${SRC_FILES}
  ${HEADER_FILES}
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(CameraPublisher PUBLIC include)
target_link_libraries(CameraPublisher ${OpenCV_LIBS})

find_package(PkgConfig)


pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)
message(STATUS "libcamera library found:")
message(STATUS "    version: ${LIBCAMERA_VERSION}")
message(STATUS "    libraries: ${LIBCAMERA_LINK_LIBRARIES}")
message(STATUS "    include path: ${LIBCAMERA_INCLUDE_DIRS}")

pkg_check_modules(LIBEVENT REQUIRED IMPORTED_TARGET libevent_pthreads)
message(STATUS "libevent_pthreads library found:")
message(STATUS "    version: ${LIBEVENT_VERSION}")
message(STATUS "    libraries: ${LIBEVENT_LINK_LIBRARIES}")
message(STATUS "    include path: ${LIBEVENT_INCLUDE_DIRS}")

target_link_libraries(CameraPublisher PkgConfig::LIBEVENT)
target_link_libraries(CameraPublisher PkgConfig::LIBCAMERA)