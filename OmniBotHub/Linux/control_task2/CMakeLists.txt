cmake_minimum_required(VERSION 2.6)
SET(PROJECT_NAME control_task_tinker)
PROJECT(${PROJECT_NAME})
#find_package(Eigen3)

# set( CMAKE_BUILD_TYPE "Release" )
# add_definitions(-std=c++17)
add_compile_options(-fPIC)
add_compile_definitions(__USE_SINGLE_PRECISION__)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
set( CMAKE_BUILD_TYPE "Release" )
set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -pthread -march=rv64gcvh" )
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../build)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -rdynamic")
set(CMAKE_COLOR_MAKEFILE ON)

# 添加 spacemit-onnxruntime 库
set(ONNXRUNTIME_INCLUDE_DIR "/usr/include")
set(ONNXRUNTIME_LIB_DIR "/usr/lib")
include_directories(${ONNXRUNTIME_INCLUDE_DIR})
link_directories(${ONNXRUNTIME_LIB_DIR})

# 设置 CycloneDDS 库
set(CycloneDDS-CXX_DIR "/home/bianbu-tinker/DDS/cyclonedds-c-cxx/cyclonedds-cxx/lib/cmake/CycloneDDS-CXX")
set(CycloneDDS_DIR  "/home/bianbu-tinker/DDS/cyclonedds-c-cxx/cyclonedds/lib/cmake/CycloneDDS")
# 设置 idlc 路径
set(_idlc_executable  "/home/bianbu-tinker/DDS/cyclonedds-c-cxx/cyclonedds/bin/idlc")
# Find CycloneDDS
find_package(CycloneDDS-CXX REQUIRED)
find_package(CycloneDDS REQUIRED)

#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>from openloong start>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
set(dirUtil "${CMAKE_CURRENT_SOURCE_DIR}/loong_utility")
set(dirqpOASES ${CMAKE_SOURCE_DIR}/qpOASES/include)
set(allInc ${dirUtil} ${dirUtil}/eigen3 ${dirqpOASES})
# 第三方.h
set(incPreFix ${CMAKE_SOURCE_DIR}/loong_ctrl/third_party/include)
file(GLOB children RELATIVE ${incPreFix} ${incPreFix}/*)
foreach(child ${children})
        if(IS_DIRECTORY ${incPreFix}/${child})
                list(APPEND allInc ${incPreFix}/${child})
        endif()
endforeach()
# 项目.h
set(srcPreFix ${CMAKE_SOURCE_DIR}/loong_ctrl/src)
file(GLOB children RELATIVE ${srcPreFix} ${srcPreFix}/*)
foreach(child ${children})
        if(IS_DIRECTORY ${srcPreFix}/${child})
                list(APPEND allInc ${srcPreFix}/${child})
        endif()
endforeach()

include_directories(${allInc})
# ================================
message("=== linux riscv64架构 ===")
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/loong_ctrl/third_party/lib_lin_riscv64")
set(tinker tinker_riscv64)
# ================================
# 设置 RPATH 为相对于可执行文件的 ./lib_files 目录
set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/lib_files")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_SKIP_BUILD_RPATH FALSE)
# 确保 CMake 使用指定的 RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

file(GLOB utilFiles ${dirUtil}/*.cpp)
add_library(util STATIC ${utilFiles})
# -----------
file(GLOB_RECURSE srcFiles "./loong_ctrl/src/*.cpp")
add_library(${tinker} SHARED ${srcFiles})

set(coreLibs util)

target_link_libraries(${tinker} ${coreLibs} pthread)

include_directories( 
     "./build"
     "./inc"
     "./math_inc"
     "./vmc_inc"
     "./mpc_locomotion"
     "/usr/local/include"
)

link_directories("/usr/local/lib")
include_directories("/usr/local/include")

AUX_SOURCE_DIRECTORY(src DIR_SRCS)
AUX_SOURCE_DIRECTORY(math_src DIR_SRCS1)
AUX_SOURCE_DIRECTORY(vmc_src DIR_SRCS2)
AUX_SOURCE_DIRECTORY(gait_src DIR_SRCS3)
AUX_SOURCE_DIRECTORY(Param DIR_SRCS4)
AUX_SOURCE_DIRECTORY(mpc_locomotion DIR_SRCS5)
AUX_SOURCE_DIRECTORY(vision_location DIR_SRCS6)

# Generate WalkCommand IDL
idlcxx_generate(TARGET WalkCommand FILES "WalkCommand.idl" WARNINGS no-implicit-extensibility)

ADD_EXECUTABLE(${PROJECT_NAME} ${DIR_SRCS} ${DIR_SRCS1}  ${DIR_SRCS2} ${DIR_SRCS3}  ${DIR_SRCS4} ${DIR_SRCS5} ${DIR_SRCS6})
target_link_libraries(${PROJECT_NAME} ${tinker} onnxruntime spacemit_ep CycloneDDS-CXX::ddscxx WalkCommand)

set(CMAKE_COLOR_MAKEFILE ON)
# set(CMAKE_CXX_STANDARD 14)

add_subdirectory(qpOASES lib/ EXCLUDE_FROM_ALL)
target_link_libraries(${PROJECT_NAME} qpOASES)

add_subdirectory(yaml-cpp lib/ EXCLUDE_FROM_ALL)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

#add_subdirectory(eigen lib/ EXCLUDE_FROM_ALL)
#target_link_libraries(${PROJECT_NAME} eigen)
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION .)
install(TARGETS ${tinker} LIBRARY DESTINATION ./lib_files)
