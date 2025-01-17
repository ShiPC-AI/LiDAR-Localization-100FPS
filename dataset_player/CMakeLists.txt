add_executable(play_kitti_for_mapping play_kitti_for_mapping.cpp)
target_link_libraries(play_kitti_for_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES})


add_executable(play_kitti_for_relocate play_kitti_for_relocate.cpp)
target_link_libraries(play_kitti_for_relocate ${catkin_LIBRARIES} ${PCL_LIBRARIES})