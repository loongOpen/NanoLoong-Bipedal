cmake_minimum_required(VERSION 3.22)

project(sim2sim)

set(CMAKE_CXX_STANDARD 17)

# 查找Fast DDS依赖包
set(fastrtps_DIR "/home/bianbu/DDS/FAST/Fast-DDS/install/share/fastrtps/cmake")
set(fastcdr_DIR "/home/bianbu/DDS/FAST/Fast-CDR/install/lib/cmake/fastcdr")
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
# message(STATUS "fastcdr_FOUND = ${fastcdr_FOUND}")
# message(STATUS "fastcdr_DIR = ${fastcdr_DIR}")
# message(STATUS "fastrtps_FOUND = ${fastrtps_FOUND}")
# message(STATUS "fastrtps_DIR = ${fastrtps_DIR}")

# 添加生成的头文件目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Request
    ${CMAKE_CURRENT_SOURCE_DIR}/Response
)
# 创建消息类型库：Request
set(REQUEST_SOURCES
    Request/Request.cxx
    Request/RequestPubSubTypes.cxx
)
add_library(RequestLib SHARED ${REQUEST_SOURCES})
target_link_libraries(RequestLib PUBLIC fastcdr fastrtps)

# 创建消息类型库：Response
set(RESPONSE_SOURCES
    Response/Response.cxx
    Response/ResponsePubSubTypes.cxx
)
add_library(ResponseLib SHARED ${RESPONSE_SOURCES})
target_link_libraries(ResponseLib PUBLIC fastcdr fastrtps)

# 添加onnxruntime库
set(ONNXRUNTIME_INCLUDE_DIR "/usr/include")
set(ONNXRUNTIME_LIB_DIR "/usr/lib")

# 添加eigen库
find_package(Eigen3 REQUIRED)

include_directories(${ONNXRUNTIME_INCLUDE_DIR})
link_directories(${ONNXRUNTIME_LIB_DIR})

# 包含目录
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src) 
include_directories(${PROJECT_SOURCE_DIR}/loong_utility)

# 添加可执行文件
add_executable(rl_gait_switch_dds src/rl_gait_switch_dds.cpp src/rl_onnx.cpp)

# target_include_directories(rl_gait_switch_dds PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
# 链接ONNX Runtime库
target_link_libraries(rl_gait_switch_dds PUBLIC onnxruntime spacemit_ep RequestLib ResponseLib fastcdr fastrtps)