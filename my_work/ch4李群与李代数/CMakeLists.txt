cmake_minimum_required( VERSION 2.8 )
project( useSophus )

include_directories( "/usr/include/eigen3" )
# 为使用 sophus，您需要使用find_package命令找到它
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable( useSophus useSophus.cpp )
target_link_libraries( useSophus ${Sophus_LIBRARIES} )

# Sophus/Eigen 的类里有 SSE/AVX 对齐要求，错配就会导致“corrupted double-linked list”这种堆链表损坏。
add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY)

option(USE_UBUNTU_20 "Set to ON if you are using Ubuntu 20.04" OFF)
find_package(Pangolin REQUIRED)
if(USE_UBUNTU_20)
    message("You are using Ubuntu 20.04, fmt::fmt will be linked")
    find_package(fmt REQUIRED)
    set(FMT_LIBRARIES fmt::fmt)
endif()
include_directories(${Pangolin_INCLUDE_DIRS})
add_executable(trajectoryError trajectoryError.cpp)
target_link_libraries(trajectoryError ${Pangolin_LIBRARIES} ${FMT_LIBRARIES})

