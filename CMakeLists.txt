cmake_minimum_required(VERSION 3.16)
project(NavigationSystem VERSION 1.0.0 LANGUAGES CXX)

# Set C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MSVC 使用 UTF-8 编码源文件
if(MSVC)
    add_compile_options(/utf-8)
endif()

# Enable Qt MOC, UIC, and RCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Find Qt5 modules
find_package(Qt5 5.14 REQUIRED COMPONENTS Core Widgets)

# Add source directory
add_subdirectory(src)
