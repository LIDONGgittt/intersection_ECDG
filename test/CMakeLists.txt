## setup tests
include(GoogleTest)
file(GLOB test_files "*_test.cpp")
message("#### Test files are: " ${test_files})

foreach(file ${test_files})
    # string( REPLACE ".cpp" "" testname ${file} )
    get_filename_component(testname ${file} NAME_WE)
    add_executable(${testname} ${file})
    target_link_libraries(${testname} ${PROJECT_LIB_NAME} GTest::gtest_main gtest gmock)
    gtest_discover_tests(${testname})
endforeach()


file(GLOB main_files "main*.cpp")
message("#### Main files are: " ${main_files})

foreach(file ${main_files})
    # string( REPLACE ".cpp" "" testname ${file} )
    get_filename_component(filename ${file} NAME_WE)
    add_executable(${filename} ${file})
    target_link_libraries(${filename} ${PROJECT_LIB_NAME} yaml-cpp)
endforeach()