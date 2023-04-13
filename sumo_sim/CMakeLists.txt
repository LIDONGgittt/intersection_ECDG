file(GLOB sumo_files "sumo_sim_*.cpp")
message("#### SUMO simulation files are: " ${sumo_files})

foreach(file ${sumo_files})
    # string( REPLACE ".cpp" "" testname ${file} )
    get_filename_component(simname ${file} NAME_WE)
    add_executable(${simname} ${file})
    target_link_libraries(${simname} ${PROJECT_LIB_NAME} SUMO::libtracicpp)
endforeach()