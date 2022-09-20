cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_BUILD_TYPE Debug)

project(Intersection_CDG VERSION 0.1.0)
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

file(GLOB SOURCES "src/*.cpp")

add_executable(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} 
    PRIVATE
    ${PROJECT_SOURCE_DIR}/include)
