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
    "src/osm/common.hpp"
    "src/osm/way_network.hpp"
    "src/osm/aabb_tree.hpp"
    "src/osm/geom.hpp"
    "src/osm/geom.cpp"
    "src/osm/mesh.hpp"
    "src/osm/mesh.cpp"
    "src/osm/osm.hpp"
    "src/osm/osm.cpp"
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

if (WIN32)
    target_compile_definitions(3dcity PUBLIC NOMINMAX)
endif()

find_path(EARCUT_HPP_INCLUDE_DIRS "mapbox/earcut.hpp")
target_include_directories(3dcity PRIVATE ${EARCUT_HPP_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS geometry)
target_link_libraries(3dcity PRIVATE Boost::geometry)

find_package(Boost REQUIRED COMPONENTS container)
target_link_libraries(3dcity PRIVATE Boost::container)

find_package(CGAL CONFIG REQUIRED)
target_link_libraries(3dcity PRIVATE CGAL::CGAL)

#find_package(CGAL COMPONENTS CGAL::Qt6)
#target_compile_definitions(3dcity CGAL_USE_BASIC_VIEWER=1)
#target_link_libraries(3dcity PUBLIC CGAL::Qt6)

# -------------------------------- OSMIUM ------------------------------------

find_path(OSMIUM_INCLUDE_DIRS "osmium/version.hpp")
target_include_directories(3dcity PRIVATE ${OSMIUM_INCLUDE_DIRS})

find_package(BZip2 REQUIRED)
target_link_libraries(3dcity PRIVATE BZip2::BZip2)

find_package(ZLIB REQUIRED)
target_link_libraries(3dcity PRIVATE ZLIB::ZLIB)

find_package(EXPAT REQUIRED)
target_link_libraries(3dcity PUBLIC EXPAT::EXPAT)

# -----------------------------------------------------------------------------

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

find_path(SCOTTT_DEBUGBREAK_INCLUDE_DIRS "debugbreak.h")
target_include_directories(3dcity PRIVATE ${SCOTTT_DEBUGBREAK_INCLUDE_DIRS})