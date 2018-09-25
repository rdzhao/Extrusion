cmake_minimum_required(VERSION 3.1)

project(Extrusion)

add_subdirectory(glm)

file(GLOB Headers "src/*.h")
file(GLOB Sources "src/*.cpp")

add_executable(Extrusion ${Headers} ${Sources})

target_include_directories(Extrusion PUBLIC glm)