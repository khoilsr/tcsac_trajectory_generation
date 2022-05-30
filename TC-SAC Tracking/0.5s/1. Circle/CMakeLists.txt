project(SingleTrackModel)
cmake_minimum_required(VERSION 2.8)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

aux_source_directory(. SRC_LIST)
add_executable(main ${SRC_LIST})

target_link_libraries(main armadillo)
