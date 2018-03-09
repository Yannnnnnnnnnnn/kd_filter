CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

project(kd_filter C)

add_executable(kd_filter kdtree.h kdtree.c main.c)
target_link_libraries(kd_filter -lm)