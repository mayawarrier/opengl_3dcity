
set(ASSETS_SRC_DIR "${SRC_DIR}/assets")
set(ASSETS_DST_DIR "${DST_DIR}/assets")
set(ASSET_TIMESTAMP_FILE "${DST_DIR}/assets.timestamp")

file(MAKE_DIRECTORY "${ASSETS_DST_DIR}")

file(GLOB_RECURSE ASSET_FILES
    RELATIVE "${ASSETS_SRC_DIR}"
    "${ASSETS_SRC_DIR}/*")

foreach(REL_PATH IN LISTS ASSET_FILES)
    set(SRC_FILE "${ASSETS_SRC_DIR}/${REL_PATH}")
    set(DST_FILE "${ASSETS_DST_DIR}/${REL_PATH}")
    get_filename_component(FILE_DST_DIR "${DST_FILE}" DIRECTORY)

    file(MAKE_DIRECTORY "${FILE_DST_DIR}")

    if (NOT EXISTS "${DST_FILE}" OR "${SRC_FILE}" IS_NEWER_THAN "${ASSET_TIMESTAMP_FILE}")
        message(STATUS "Copying ${SRC_FILE}")
        file(COPY "${SRC_FILE}" DESTINATION "${FILE_DST_DIR}")
    endif()
endforeach()

file(TOUCH "${ASSET_TIMESTAMP_FILE}")
