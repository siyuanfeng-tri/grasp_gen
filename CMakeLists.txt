cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(grasp_gen)
find_package(PCL REQUIRED)

include(ExternalProject)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${CMAKE_INSTALL_PREFIX}/include
  ${PCL_INCLUDE_DIRS})
link_directories(
  ${CMAKE_INSTALL_PREFIX}/lib)

ExternalProject_Add(gpg_modified
  GIT_REPOSITORY https://github.com/siyuanfeng-tri/gpg_modified.git
  GIT_TAG afdbacd
  CMAKE_ARGS
  -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_definitions(
  -DGRASP_PARAM_PATH="${PROJECT_SOURCE_DIR}/../cfg/grasp_params")

add_library(grasp_gen SHARED
  src/anti_podal_grasp.cc)
add_dependencies(grasp_gen gpg_modified)
target_link_libraries(grasp_gen ${PCL_LIBRARIES} grasp_candidates_generator libgpg_config_file.a)

add_executable(test_grasp src/test.cc)
target_link_libraries(test_grasp grasp_gen)

install(TARGETS grasp_gen
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

install(TARGETS test_grasp
  DESTINATION ${CMAKE_INSTALL_PREFIX})

install(DIRECTORY include/grasp_gen
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
