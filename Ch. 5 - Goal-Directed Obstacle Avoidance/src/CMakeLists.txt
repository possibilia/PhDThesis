cmake_minimum_required(VERSION 3.10)

project(ObstacleAvoid)

find_package(Threads)
find_package(CURL REQUIRED) 

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_compile_options(-Wall)

include(GNUInstallDirs)

find_library(alphabot NAMES alphabot HINTS "${CMAKE_INSTALL_LIBDIR}")
find_library(a1lidarrpi NAMES a1lidarrpi HINTS "${CMAKE_INSTALL_LIBDIR}")

include_directories(${CURL_INCLUDE_DIR})

add_subdirectory(agent)

add_executable(autoctrl autoctrl.cpp)
target_link_libraries(autoctrl alphabot a1lidarrpi pigpio pthread agent ncurses)
