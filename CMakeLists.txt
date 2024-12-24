cmake_minimum_required(VERSION 3.16)
project(igh)
# 源文件
file(GLOB SRC_LIST "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp" "${CMAKE_SOURCE_DIR}/humanoid_sharedmemory/*.cpp")

# 设置库文件输出路径
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
# 查找ethercat库
find_library(ETHERCAT_LIBRARY ethercat PATHS /opt/etherlab/lib)
# 生成igh库
add_library(ighLib STATIC ${SRC_LIST})
# 增加头文件
target_include_directories(ighLib PUBLIC 
                        ${CMAKE_CURRENT_SOURCE_DIR}
                        ${CMAKE_SOURCE_DIR}/include/humanoid_sharedmemory
                        ${CMAKE_SOURCE_DIR}/include/humanoid_hardware/motor_communication
)
# 添加所需要的库文件
target_link_libraries(ighLib ${ETHERCAT_LIBRARY} pthread SharedMemory)
