add_executable(run_global_localization run_global_localization.cpp)
target_link_libraries(run_global_localization ${PCL_LIBRARIES}
                                ${LL_100FPS_EXTRACTOR_LIB}
                                ${LL_100FPS_LOCALIZER_LIB}
                                ${LL_100FPS_BASE_LIB})