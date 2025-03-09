cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories("/home/izumi/libs/eigen")

add_executable(Rasterizer main.cpp OBJ_Loader.h Shader.hpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp Texture.hpp Texture.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})