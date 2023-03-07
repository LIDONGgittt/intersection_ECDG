file(GLOB batch_files "batch_test_*.cpp")
message("#### Batch test files are: " ${batch_files})

foreach(file ${batch_files})
    # string( REPLACE ".cpp" "" testname ${file} )
    get_filename_component(batchname ${file} NAME_WE)
    add_executable(${batchname} ${file})
    target_link_libraries(${batchname} ${PROJECT_LIB_NAME})
endforeach()