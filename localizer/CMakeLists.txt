SET(LIB_NAME ll_100fps_localizer)
SET(LIB_NAME_UPPER LL_100FPS_LOCALIZER)

aux_source_directory(./ ${LIB_NAME_UPPER}_SRC)
SET(${LIB_NAME_UPPER}_LIB ${LIB_NAME} CACHE INTERNAL "lib")

ADD_LIBRARY(${${LIB_NAME_UPPER}_LIB} SHARED 
            ${${LIB_NAME_UPPER}_SRC})

# Opencv 4.5.4
TARGET_LINK_LIBRARIES(${${LIB_NAME_UPPER}_LIB} 
                       ${PCL_LIBRARIES}
                    #    ${OpenCV_LIBS}
                    libLL_100FPS_tree.so
                    ${LL_100FPS_BASE_LIB})
