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