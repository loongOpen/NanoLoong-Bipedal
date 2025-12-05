# cmake_minimum_required(VERSION 3.25)
cmake_minimum_required(VERSION 3.22)

project(sim2sim)

set(CMAKE_CXX_STANDARD 17)

# 添加LCM库
set(LCM_DIR "/usr/local/lib/lcm/cmake")
find_package(lcm REQUIRED)

set(LCM_INCLUDE_DIRS "/usr/local/include")
include_directories(${LCM_INCLUDE_DIRS})
set(LCM_LIBRARY_DIRS "/usr/local/lib")
link_directories(${LCM_LIBRARY_DIRS})

# 添加onnxruntime库
set(ONNXRUNTIME_INCLUDE_DIR "/usr/include")
set(ONNXRUNTIME_LIB_DIR "/usr/lib")

# 添加eigen库
find_package(Eigen3 REQUIRED)

include_directories(${ONNXRUNTIME_INCLUDE_DIR})
link_directories(${ONNXRUNTIME_LIB_DIR})

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/lcm_types) 
include_directories(${PROJECT_SOURCE_DIR}/src) 
include_directories(${PROJECT_SOURCE_DIR}/loong_utility)

add_executable(lcm_publisher_onnx src/lcm_publisher_onnx.cpp)
add_executable(rl_gait_switch_lcm src/rl_gait_switch_lcm.cpp src/rl_onnx.cpp)

target_link_libraries(lcm_publisher_onnx lcm)
target_link_libraries(rl_gait_switch_lcm onnxruntime lcm)
target_link_libraries(lcm_publisher_onnx onnxruntime spacemit_ep) # 依赖onnxruntime库
target_link_libraries(rl_gait_switch_lcm onnxruntime spacemit_ep)