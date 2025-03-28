cmake_minimum_required(VERSION 3.10.0) 
project(Maze)

add_executable(app "${PROJECT_SOURCE_DIR}/main.cpp")
target_link_libraries(app PRIVATE SDL2)
target_link_libraries(app PRIVATE SDL2_gfx)
