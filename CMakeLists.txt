cmake_minimum_required(VERSION 3.14)
project(IntersectionManagement VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE Release)

## fetch google test
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/release-1.12.1.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

find_package(yaml-cpp REQUIRED)
# find_package(Boost REQUIRED)

include_directories(${YAML_CPP_INCLUDE_DIR})
# include_directories(${Boost_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS include/*.h)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

list(FILTER SOURCES EXCLUDE REGEX "src/cdg_main.cpp")
list(FILTER SOURCES EXCLUDE REGEX "src/main.cpp")
set(LIB_SOURCES ${SOURCES})
set(PROJECT_LIB_NAME ${PROJECT_NAME}_lib)
add_library(${PROJECT_LIB_NAME} STATIC ${LIB_SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_LIB_NAME} ${YAML_CPP_LIBRARIES})

## setup tests
enable_testing()
add_subdirectory(test)