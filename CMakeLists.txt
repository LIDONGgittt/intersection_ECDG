cmake_minimum_required(VERSION 3.14)
project(Intersection_CDG VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_BUILD_TYPE Debug)

## fetch google test
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG 58d77fa8070e8cec2dc1ed015d66b454c8d78850
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# find_package(Boost REQUIRED)
# include_directories(${Boost_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS include/*.h)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
# target_include_directories(${PROJECT_NAME} 
#     PRIVATE
#     ${PROJECT_SOURCE_DIR}/include)
list(FILTER SOURCES EXCLUDE REGEX "src/main.cpp")
set(LIB_SOURCES ${SOURCES})
set(PROJECT_LIB_NAME ${PROJECT_NAME}_lib)
add_library(${PROJECT_LIB_NAME} STATIC ${LIB_SOURCES} ${HEADERS})

## setup tests
enable_testing()
add_subdirectory(test)