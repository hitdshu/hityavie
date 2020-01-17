cmake_minimum_required(VERSION 3.1.3)
project(hityavie)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE Release)
set(OpenCV_DIR /usr/local/share/OpenCV)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Ceres REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${EIGEN3_INCLUDE_DIR}
${PROTOBUF_INCLUDE_DIRS}
${CERES_INCLUDE_DIRS}
${Pangolin_INCLUDE_DIRS}
)

exec_program(protoc ARGS "--cpp_out=${PROJECT_SOURCE_DIR}/proto/ --proto_path=${PROJECT_SOURCE_DIR}/proto/ ${PROJECT_SOURCE_DIR}/proto/*.proto")

file(GLOB CAMERA_CC "${PROJECT_SOURCE_DIR}/camera/*.cc")
file(GLOB COMMON_CC "${PROJECT_SOURCE_DIR}/common/*.cc")
file(GLOB IMU_CC "${PROJECT_SOURCE_DIR}/imu/*.cc")
file(GLOB MAP_CC "${PROJECT_SOURCE_DIR}/map/*.cc")
file(GLOB PROTO_CC "${PROJECT_SOURCE_DIR}/proto/*.cc")
file(GLOB TRACKER_CC "${PROJECT_SOURCE_DIR}/tracker/*.cc")
file(GLOB YAVIE_CC "${PROJECT_SOURCE_DIR}/yavie/*.cc")

add_library(${PROJECT_NAME} SHARED
${CAMERA_CC}
${COMMON_CC}
${IMU_CC}
${MAP_CC}
${PROTO_CC}
${TRACKER_CC}
${YAVIE_CC}
)
target_link_libraries(${PROJECT_NAME}
${EIGEN3_LIBS}
${OpenCV_LIBS}
${PROTOBUF_LIBRARIES}
${CERES_LIBRARIES}
${Pangolin_LIBRARIES}
)

add_executable(Test test.cc)
target_link_libraries(Test
${PROJECT_NAME}
)
