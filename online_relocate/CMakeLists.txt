add_executable(load_map load_map.cpp)
target_link_libraries(load_map ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(relocate relocate.cpp)
target_link_libraries(relocate ${catkin_LIBRARIES} ${PCL_LIBRARIES})
