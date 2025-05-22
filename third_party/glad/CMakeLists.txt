
# This file is not part of the glad/Khronos/EGL codebase, 
# and is not covered by the glad LICENSE.txt.

cmake_minimum_required(VERSION 3.15)
project(glad)

add_library(glad STATIC "src/glad.c")
target_include_directories(glad PUBLIC "include/")