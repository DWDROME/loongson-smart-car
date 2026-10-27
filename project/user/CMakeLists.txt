cmake_minimum_required(VERSION 3.5.0)

# 交叉编译工具链
include(cross.cmake)

# 基础路径与项目名称配置
set(CURRENT_DIR ${CMAKE_CURRENT_LIST_DIR})
get_filename_component(PARENT_DIR ${CURRENT_DIR}/.. ABSOLUTE)  
get_filename_component(PROJ_NAME ${PARENT_DIR} NAME)           # 取上级目录名作为项目名
project(${PROJ_NAME} VERSION 0.1.0 LANGUAGES C CXX)

# 编译参数（龙芯LoongArch适配）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 通用编译优化与龙芯架构参数
set(COMMON_FLAGS "-O3 -march=loongarch64 -mtune=loongarch64 -fPIC -ffunction-sections -fdata-sections -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS} -pthread -Wno-delete-non-virtual-dtor -Wno-placement-new")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")

# OpenCV依赖配置
set(CMAKE_PREFIX_PATH "/opt/ls_2k0300_env/opencv_4_10_build") 
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_INCLUDE: ${OpenCV_INCLUDE_DIRS}, OpenCV_LIBS: ${OpenCV_LIBS}")

# 头文件路径汇总（运行时分类后端固定为仓库内 NCNN）
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    # 业务代码头文件
    ../user 
    ../code 
    ../model
    ../../libraries/zf_common 
    ../../libraries/zf_device 
    ../../libraries/zf_driver
    ../../libraries/zf_components/seekfree_assistant
    
    ../../libraries/zf_components/ncnn/include/ncnn
)

# 静态库路径
link_directories(
    ../../libraries/zf_components/ncnn/lib
    ../../libraries/lq
)

# 业务源文件（tools/ 子目录不进主程序 aux）
aux_source_directory(../user DIR_SRCS)
aux_source_directory(../code DIR_SRCS)
aux_source_directory(../model DIR_SRCS)
aux_source_directory(../../libraries/zf_common DIR_SRCS)
aux_source_directory(../../libraries/zf_device DIR_SRCS)
aux_source_directory(../../libraries/zf_driver DIR_SRCS)
aux_source_directory(../../libraries/zf_components/seekfree_assistant DIR_SRCS)
# project/code 子目录不会被 aux_source_directory 递归收集。
list(APPEND DIR_SRCS
    ../code/detour/red_brick_filter.cpp
    ../code/detour/red_event_router.cpp
    ../code/detour/red_brick_detour.cpp
    ../code/detour/red_brick_detour_runtime.cpp
)

# TFLite 源码和模型保留作显式回滚产物，但主程序只有 NCNN 后端。
set(NCNN_DIR_SRCS)
foreach(SOURCE_FILE ${DIR_SRCS})
    get_filename_component(SOURCE_NAME "${SOURCE_FILE}" NAME)
    if(NOT SOURCE_NAME STREQUAL "mytflite.cpp")
        list(APPEND NCNN_DIR_SRCS "${SOURCE_FILE}")
    endif()
endforeach()
set(DIR_SRCS ${NCNN_DIR_SRCS})
message(STATUS "Vision classifier backend: NCNN only")

# 生成可执行文件 & 配置属性
add_executable(${PROJ_NAME} ${DIR_SRCS})
set_target_properties(${PROJ_NAME} PROPERTIES
    INSTALL_RPATH "/opt/ls_2k0300_env/opencv_4_10_build/lib"
    BUILD_WITH_INSTALL_RPATH ON
)

# 链接依赖库
target_link_libraries(
${PROJ_NAME} 
${OpenCV_LIBS}
pthread
m
ncnn
gomp
)

# ---------------------------------------------------------------------------
# 独立硬件工具（不链 OpenCV/NCNN/主业务，避免塞进主二进制）
# ---------------------------------------------------------------------------
set(TOOL_COMMON_INCLUDES
    ../../libraries/zf_common
    ../../libraries/zf_device
    ../../libraries/zf_driver
)
set(TOOL_RPATH "/opt/ls_2k0300_env/opencv_4_10_build/lib")

function(add_hw_tool name)
    add_executable(${name} ${ARGN})
    target_include_directories(${name} PRIVATE ${TOOL_COMMON_INCLUDES})
    target_link_libraries(${name} PRIVATE m pthread)
    set_target_properties(${name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tools
        BUILD_WITH_INSTALL_RPATH ON
        INSTALL_RPATH ${TOOL_RPATH}
    )
endfunction()

# 驱动源：工具只链需要的 .cpp
set(DRV_PWM ../../libraries/zf_driver/zf_driver_pwm.cpp)
set(DRV_GPIO ../../libraries/zf_driver/zf_driver_gpio.cpp)
set(DRV_ENC ../../libraries/zf_driver/zf_driver_encoder.cpp)
set(DRV_DELAY ../../libraries/zf_driver/zf_driver_delay.cpp)
set(DRV_FS ../../libraries/zf_driver/zf_driver_file_string.cpp)
set(DRV_FB ../../libraries/zf_driver/zf_driver_file_buffer.cpp)
set(DEV_IMU ../../libraries/zf_device/zf_device_imu.cpp)

add_hw_tool(motor_spin
    tools/motor_spin.cpp ${DRV_PWM} ${DRV_GPIO} ${DRV_DELAY} ${DRV_FS} ${DRV_FB})
add_hw_tool(encoder_watch
    tools/encoder_watch.cpp ${DRV_ENC} ${DRV_DELAY} ${DRV_FB} ${DRV_FS})
add_hw_tool(imu_watch
    tools/imu_watch.cpp ${DEV_IMU} ${DRV_DELAY} ${DRV_FS} ${DRV_FB})
add_hw_tool(key_supervisor
    tools/key_supervisor.cpp ${DRV_GPIO} ${DRV_PWM} ${DRV_DELAY} ${DRV_FS} ${DRV_FB})

# 目标板 NCNN smoke/parity：复用主程序同一分类适配器，不接相机和控制。
add_executable(ncnn_model_smoke
    tools/ncnn_model_smoke.cpp
    ../code/lq_ncnn.cpp
)
target_include_directories(ncnn_model_smoke PRIVATE
    ../code
    ../../libraries/zf_components/ncnn/include/ncnn
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(ncnn_model_smoke PRIVATE
    ${OpenCV_LIBS}
    ncnn
    gomp
    m
    pthread
)
set_target_properties(ncnn_model_smoke PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tools
    BUILD_WITH_INSTALL_RPATH ON
    INSTALL_RPATH ${TOOL_RPATH}
)

# 板端离线评估：复用主程序同一 NCNN 适配器，扫描 supplies/vehicle/weapon 数据集。
add_executable(ncnn_dataset_eval
    tools/ncnn_dataset_eval.cpp
    ../code/lq_ncnn.cpp
)
target_include_directories(ncnn_dataset_eval PRIVATE
    ../code
    ../../libraries/zf_components/ncnn/include/ncnn
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(ncnn_dataset_eval PRIVATE
    ${OpenCV_LIBS}
    ncnn
    gomp
    m
    pthread
)
set_target_properties(ncnn_dataset_eval PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tools
    BUILD_WITH_INSTALL_RPATH ON
    INSTALL_RPATH ${TOOL_RPATH}
)

# 独立相机定频采集：只保存原始 320x240 BGR，不加载模型或控制状态机。
add_executable(camera_burst_capture
    tools/camera_burst_capture.cpp
    ../../libraries/zf_device/zf_device_uvc.cpp
)
target_include_directories(camera_burst_capture PRIVATE
    ../code
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(camera_burst_capture PRIVATE
    ${OpenCV_LIBS}
    liblq_camera_ex.a
    m
    pthread
)
set_target_properties(camera_burst_capture PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tools
    BUILD_WITH_INSTALL_RPATH ON
    INSTALL_RPATH ${TOOL_RPATH}
)

# 只读诊断：比较原包示例 64x64 与其推理类默认 96x96，主程序不调用。
add_executable(tiny_classifier_size_probe
    tools/tiny_classifier_size_probe.cpp
)
target_include_directories(tiny_classifier_size_probe PRIVATE
    ../../libraries/zf_components/ncnn/include/ncnn
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(tiny_classifier_size_probe PRIVATE
    ${OpenCV_LIBS}
    ncnn
    gomp
    m
    pthread
)
set_target_properties(tiny_classifier_size_probe PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tools
    BUILD_WITH_INSTALL_RPATH ON
    INSTALL_RPATH ${TOOL_RPATH}
)

# Board/Brick 路由和绕行状态机纯逻辑 smoke，不链接相机、OpenCV、NCNN 或电机。
add_executable(red_event_router_smoke
    tools/red_event_router_smoke.cpp
    ../code/detour/red_event_router.cpp
)
target_include_directories(red_event_router_smoke PRIVATE ../code)
set_target_properties(red_event_router_smoke PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tools
)

add_executable(red_brick_detour_smoke
    tools/red_brick_detour_smoke.cpp
    ../code/detour/red_brick_detour.cpp
    ../code/detour/red_brick_detour_runtime.cpp
)
target_include_directories(red_brick_detour_smoke PRIVATE ../code)
set_target_properties(red_brick_detour_smoke PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tools
)

add_executable(race_recognition_result_smoke
    tools/race_recognition_result_smoke.cpp
    ../code/race_recognition_result.cpp
)
target_include_directories(race_recognition_result_smoke PRIVATE ../code)
set_target_properties(race_recognition_result_smoke PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tools
)
