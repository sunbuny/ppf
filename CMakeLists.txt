cmake_minimum_required(VERSION 2.8)
project(ppf3Detector)

set(CMAKE_CXX_STANDARD 11)
#SET( CMAKE_CXX_FLAGS "-fopenmp -O3 -std=c++11" )
#SET(CMAKE_CXX_COMPILER "g++")
#add_compile_options(-fopenmp)
#set(CMAKE_CXX_FLAGS "-fopenmp")

# 设定可执行二进制文件的目录
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# 设定存放编译出来的库文件目录
SET( LIBRARY_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/lib)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

# 将库文件目录设置为链接目录
LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)

# 设定头文件目录
INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include)

ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src)
