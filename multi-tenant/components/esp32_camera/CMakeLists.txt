set(CAMERA_LIB_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/libesp32-camera.a)

idf_component_register(
    SRCS "empty.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer)

target_link_libraries(${COMPONENT_LIB} INTERFACE ${CAMERA_LIB_PATH})
