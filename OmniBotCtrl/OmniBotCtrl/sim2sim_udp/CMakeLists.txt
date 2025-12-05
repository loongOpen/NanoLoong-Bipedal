# cmake_minimum_required(VERSION 3.25)
cmake_minimum_required(VERSION 3.22)

project(sim2sim)

set(CMAKE_CXX_STANDARD 17)

# 添加onnxruntime库
set(ONNXRUNTIME_INCLUDE_DIR "/usr/include")
set(ONNXRUNTIME_LIB_DIR "/usr/lib")

# 添加eigen库
find_package(Eigen3 REQUIRED)

include_directories(${ONNXRUNTIME_INCLUDE_DIR})
link_directories(${ONNXRUNTIME_LIB_DIR})

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src) 
include_directories(${PROJECT_SOURCE_DIR}/loong_utility)

add_executable(rl_gait_switch_udp src/rl_gait_switch_udp.cpp src/rl_onnx.cpp)
add_executable(rl_tinkerv1_udp src/rl_tinkerv1_udp.cpp src/rl_onnx.cpp)

target_link_libraries(rl_gait_switch_udp onnxruntime spacemit_ep)
target_link_libraries(rl_tinkerv1_udp onnxruntime spacemit_ep)