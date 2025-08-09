# 声明要求的 cmake 最低版本
cmake_minimum_required(VERSION 3.10)
# 声明一个 cmake 工程 
project( HelloSLAM )
# 添加一个可执行程序
add_executable( helloSLAM helloSLAM.cpp )

# 创建一个共享库文件 
add_library( hello_shared SHARED src/libhelloSLAM.cpp )

# 告诉编译器去哪里找头文件（如果你的头文件在 include 目录）
target_include_directories( hello_shared PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# 添加一个可执行程序
add_executable( useHello useHello.cpp ) 
# 链接到所需库
target_link_libraries( useHello hello_shared )