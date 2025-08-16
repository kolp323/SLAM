
cmake_minimum_required( VERSION 2.8 )
project( Eigen )
include_directories( "/usr/include/eigen3" )

#### useEigen
# find_package 不会直接修改编译选项，它只是查找并提供结果。
# REQUIRED表示如果找不到就报错
# find_package(Eigen3 REQUIRED) 
add_executable( useEigen useEigen.cpp )

#### useGeometry
add_executable( useGeometry useGeometry.cpp )

#### visulizeGeometry
# 添加Pangolin依赖
find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable( visualizeGeometry visualizeGeometry.cpp )
target_link_libraries( visualizeGeometry ${Pangolin_LIBRARIES} )