cmake_minimum_required(VERSION 3.15)
project(3dcity)


include(FetchContent)

set(SOURCES 
    "src/utils.hpp"
    "src/utils.cpp"
    "src/glutils.hpp"
    "src/glutils.cpp"
    "src/viewport.hpp"
    "src/viewport.cpp"
    "src/main.cpp")

if (WIN32)
    list(APPEND SOURCES 
        "src/win32.hpp"
        "src/win32.cpp")
endif()

if (CMAKE_GENERATOR MATCHES "Visual Studio")
    list(APPEND SOURCES "src/cpp.hint")
endif()

add_executable(3dcity "${SOURCES}")

set_target_properties(3dcity PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO)


#set(ASSET_SRCDIR "${CMAKE_SOURCE_DIR}/assets")
#set(ASSET_BINDIR "${CMAKE_BINARY_DIR}/assets")
#
#file(GLOB_RECURSE ASSET_FILES "${ASSET_SRCDIR}/*")

#add_custom_command(
#    OUTPUT "${CMAKE_BINARY_DIR}/assets.timestamp"
#    BYPRODUCTS "${ASSET_BINDIR}"
#    COMMAND "${CMAKE_COMMAND}" -E copy_directory "${ASSET_SRCDIR}" "${ASSET_BINDIR}"
#    COMMAND "${CMAKE_COMMAND}" -E touch "${CMAKE_BINARY_DIR}/assets.timestamp"
#    DEPENDS "${ASSET_FILES}"
#    VERBATIM)
#
#add_custom_target(3dcity_assets DEPENDS "${CMAKE_BINARY_DIR}/assets.timestamp")
#set_property(TARGET 3dcity_assets APPEND PROPERTY ADDITIONAL_CLEAN_FILES "${ASSET_BINDIR}") 

add_custom_target(3dcity_assets
    COMMAND "${CMAKE_COMMAND}" "-DSRC_DIR=${CMAKE_SOURCE_DIR}" 
        "-DDST_DIR=${CMAKE_BINARY_DIR}" -P "${CMAKE_SOURCE_DIR}/CopyAssets.cmake"
    VERBATIM)

add_dependencies(3dcity 3dcity_assets)

# remove console
if (MINGW)
    target_link_options(3dcity PRIVATE "-Wl,-subsystem,windows")
elseif (WIN32)
    #target_link_options(3dcity PRIVATE "/SUBSYSTEM:WINDOWS")
endif()

find_path(OSMIUM_INCLUDE_DIRS "osmium/version.hpp")
target_include_directories(3dcity PRIVATE ${OSMIUM_INCLUDE_DIRS})

find_package(SDL2 CONFIG REQUIRED)
target_link_libraries(3dcity
    PRIVATE
    $<TARGET_NAME_IF_EXISTS:SDL2::SDL2main>
    $<IF:$<TARGET_EXISTS:SDL2::SDL2>,SDL2::SDL2,SDL2::SDL2-static>)


add_subdirectory("third_party/glad")
target_link_libraries(3dcity PRIVATE glad)

target_include_directories(3dcity PRIVATE "third_party/stb/include")

find_package(glm CONFIG REQUIRED)
target_link_libraries(3dcity PRIVATE glm::glm-header-only)