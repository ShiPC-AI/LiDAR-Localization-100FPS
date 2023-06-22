add_executable(make_map_squares make_map_squares.cpp)
target_link_libraries(make_map_squares ${PCL_LIBRARIES}
                                ${LL_100FPS_EXTRACTOR_LIB}
                                ${LL_100FPS_BASE_LIB})