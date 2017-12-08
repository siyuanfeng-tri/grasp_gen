cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(grasp_gen)
find_package(PCL REQUIRED)

include(ExternalProject)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_INSTALL_PREFIX}/include
  ${PCL_INCLUDE_DIRS})
link_directories(
  ${CMAKE_INSTALL_PREFIX})

ExternalProject_Add(gpg_modified
  GIT_REPOSITORY https://github.com/siyuanfeng-tri/gpg_modified.git
  GIT_TAG 4b202f7aafb036a68fe76b141f02b4a693d39e97
  CMAKE_ARGS
  -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})

add_definitions(-std=c++1y -g -Wall -O2 -fPIC)

add_library(grasp_gen SHARED
  src/anti_podal_grasp.cc)
add_dependencies(grasp_gen gpg_modified)

add_executable(test_grasp src/test.cc)
target_link_libraries(test_grasp grasp_gen ${PCL_LIBRARIES} grasp_candidates_generator libgpg_config_file.a)

install(TARGETS grasp_gen
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

install(DIRECTORY include/grasp_gen
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
