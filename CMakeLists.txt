cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(grasp_gen)
find_package(PCL REQUIRED)

include(ExternalProject)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/build/include
  ${PCL_INCLUDE_DIRS})

ExternalProject_Add(gpg_modified
  GIT_REPOSITORY https://github.com/siyuanfeng-tri/gpg_modified.git
  GIT_TAG 818aba855236aaf5d8397d0158d8005d187e1895
  CMAKE_ARGS
  -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})

#ExternalProject_Add(
#  DEPENDS rgbd_bridge
#  SOURCE_DIR "${PROJECT_SOURCE_DIR}/src")

add_definitions(-std=c++1y -g -Wall -O2 -fPIC)
# https://stackoverflow.com/questions/37369369/compiling-pcl-1-7-on-ubuntu-16-04-errors-in-cmake-generated-makefile

add_library(grasp_gen
  src/anti_podal_grasp.cc)
add_dependencies(grasp_gen gpg_modified)
