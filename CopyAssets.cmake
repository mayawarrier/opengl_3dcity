
file(GLOB SHADER_FILES 
    LIST_DIRECTORIES false 
    "${SRC_DIR}/assets/shaders/*")

file(GLOB TEXTURE_FILES 
    LIST_DIRECTORIES false 
    "${SRC_DIR}/assets/textures/*")

set(ASSET_TIMESTAMP_FILE "${DST_DIR}/assets.timestamp")
set(SHADERS_DESTDIR "${DST_DIR}/assets/shaders")
set(TEXTURES_DESTDIR "${DST_DIR}/assets/textures")

file(MAKE_DIRECTORY 
    "${SHADERS_DESTDIR}"
    "${TEXTURES_DESTDIR}")

foreach(SHADER_FILE IN LISTS SHADER_FILES)
    get_filename_component(SHADER_FILENAME "${SHADER_FILE}" NAME)

    if (NOT EXISTS "${SHADERS_DESTDIR}/${SHADER_FILENAME}" OR 
        "${SHADER_FILE}" IS_NEWER_THAN "${ASSET_TIMESTAMP_FILE}")

        message(STATUS "Copying ${SHADER_FILE}")
        file(COPY "${SHADER_FILE}" DESTINATION "${SHADERS_DESTDIR}")
    endif()
endforeach()

foreach(TEXTURE_FILE IN LISTS TEXTURE_FILES)
    get_filename_component(TEXTURE_FILENAME "${TEXTURE_FILE}" NAME)    

    if (NOT EXISTS "${TEXTURES_DESTDIR}/${TEXTURE_FILENAME}" OR 
        "${TEXTURE_FILE}" IS_NEWER_THAN "${ASSET_TIMESTAMP_FILE}")

        message(STATUS "Copying ${TEXTURE_FILE}")
        file(COPY "${TEXTURE_FILE}" DESTINATION "${TEXTURES_DESTDIR}")
    endif()
endforeach()

file(TOUCH "${ASSET_TIMESTAMP_FILE}")