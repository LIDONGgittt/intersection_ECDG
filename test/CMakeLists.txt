## setup tests
include(GoogleTest)
file(GLOB test_files "*_test.cpp")
message("#### Test files are: " ${test_files})

foreach(file ${test_files})
    # string( REPLACE ".cpp" "" testname ${file} )
    get_filename_component(testname ${file} NAME_WE)
    add_executable(${testname} ${file})
    target_link_libraries(${testname} ${PROJECT_LIB_NAME} GTest::gtest_main)
    gtest_discover_tests(${testname})
endforeach()