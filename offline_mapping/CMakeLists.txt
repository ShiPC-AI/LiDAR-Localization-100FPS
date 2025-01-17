add_subdirectory(linefit)

add_executable(mapping mapping.cpp)
target_link_libraries(mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${LIDAR100_LINEFIT_LIB})

add_executable(collect_database collect_database.cpp)
target_link_libraries(collect_database ${catkin_LIBRARIES} ${PCL_LIBRARIES})
