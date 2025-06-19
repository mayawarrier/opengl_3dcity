
#include <osmium/index/map/flex_mem.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/area/assembler.hpp>
#include <osmium/area/multipolygon_manager.hpp>
#include <osmium/io/xml_input.hpp>
#include <osmium/visitor.hpp>
#include <osmium/geom/coordinates.hpp>
#include <osmium/geom/mercator_projection.hpp>

#include <glm/glm.hpp>

#include "mesh.hpp"
#include "osm.hpp"



// strcmp with null check
static bool str_equal(const char* str1, const char* str2)
{
    if (str1 && !str2) { return false; }
    if (str2 && !str1) { return false; }
    if (!str2 && !str1) { return true; }
    return std::strcmp(str1, str2) == 0;
}

template <typename T>
static bool parse_num_if_exists(const char* str, T& val)
{
    return str && parse_num(str, val);
}

// Every 3 values is a vertex.
template <typename T>
static glm::vec<3, T, glm::defaultp> avg_vert(const std::vector<T>& verts)
{
    glm::vec<3, T, glm::defaultp> avg(T(0));
    for (size_t i = 0; i < verts.size(); i += 3) {
        avg.x += verts[i];
        avg.y += verts[i + 1];
        avg.z += verts[i + 2];
    }
    avg /= (verts.size() / 3);
    return avg;
}


class osm_datahandler : public osmium::handler::Handler
{
public:
    void node(const osmium::Node& node)
    {
        //auto loc = osmium::geom::MercatorProjection{}(node.location());
        //node_coords.push_back(loc.x);
        //node_coords.push_back(loc.y);
        //node_coords.push_back(0.0);
    }

    void way(const osmium::Way& way)
    {
        auto& tags = way.tags();

        if (tags.has_key("highway"))
        {
            //add_polyline_from_nodes(nodes);
        }
        else
        {
            bool is_building = tags.has_key("building");
            bool is_building_part = str_equal(tags["building:part"], "yes");

            if (is_building || is_building_part) 
            {
                int levels, min_level;
                double height, min_height;

                bool has_levels    = parse_num_if_exists(tags["building:levels"], levels);
                bool has_minlevel  = parse_num_if_exists(tags["building:min_level"], min_level);
                bool has_height    = parse_num_if_exists(tags["height"], height);
                bool has_minheight = parse_num_if_exists(tags["min_height"], min_height);

                double final_height = has_height ? height : 
                    (has_levels ? (3.0 * levels) : 3.0);
                
                double final_minheight = has_minheight ? min_height : 
                    (has_minlevel ? (3.0 * min_level) : 0.0);

                m_building_assembler.add_building(way, tags["name"], 
                    is_building_part, final_minheight, final_height);
            }
        }
    }

    // The callback functions can be either static or not depending on whether
    // you need to access any member variables of the handler.
    void area(const osmium::Area& area)
    {
        const char* name = area.tags()["name"];
        //logMESSAGE("Received area %s", name);

        //const char* amenity = area.tags()["amenity"];
        //if (amenity) {
        //    // Use the center of the first outer ring. Because we set
        //    // create_empty_areas = false in the assembler config, we can
        //    // be sure there will always be at least one outer ring.
        //    const auto center = calc_center(*area.cbegin<osmium::OuterRing>());
        //
        //    print_amenity(amenity, area.tags()["name"], center);
        //}
    }

    osm_data to_osmdata()
    {
        
        auto meshes = m_building_assembler.get_draw_data();
        logMESSAGE("Num meshes %d", meshes.size());

        // traverse all meshes and collect vertices and indices, center them
        std::vector<float> verts;
        std::vector<uint32_t> line_indices;
        std::vector<uint32_t> tri_indices;

        for (const auto& mesh : meshes) 
        {
            size_t verts_startidx = verts.size() / 3;
            verts.insert(verts.end(), mesh.verts.begin(), mesh.verts.end());
            //m_line_indices.insert(m_line_indices.end(), mesh.line_indices.begin(), mesh.line_indices.end());

            for (const auto& tri_index : mesh.tri_indices) {
                tri_indices.push_back(tri_index + verts_startidx);
            }
            for (size_t i = 0; i < mesh.line_indices.size(); i += 3) {
                line_indices.push_back(mesh.line_indices[i + 0] + verts_startidx);
                line_indices.push_back(mesh.line_indices[i + 1] + verts_startidx);
                line_indices.push_back(mesh.line_indices[i + 2]);
            }

            //tri_indices.insert(tri_indices.end(), mesh.tri_indices.begin(), mesh.tri_indices.end());
        }

        auto center = avg_vert(verts);
        
        std::vector<float> verts_float;
        verts_float.resize(verts.size());
        
        for (size_t i = 0; i < verts.size(); i += 3) {
            verts_float[i + 0] = verts[i + 0] - center.x;
            verts_float[i + 1] = verts[i + 1] - center.y;
            verts_float[i + 2] = verts[i + 2] - center.z;
        }
        return {
            .verts = std::move(verts_float),
            .tri_indices = std::move(tri_indices),
            .line_indices = std::move(line_indices)
        };
    }

private:
    building_assembler m_building_assembler;

private:
    //std::vector<double> node_coords;

    //std::vector<double> m_verts;
    //std::vector<uint32_t> m_line_indices;
    //std::vector<uint32_t> m_tri_indices;

    //std::vector<building> m_buildings;
};

bool read_osmfile(const fs::path& path, osm_data& out_data)
{
    using index_t = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;
    using location_handler_t = osmium::handler::NodeLocationsForWays<index_t>;

    try {
        const osmium::io::File input_file{ path.string() };

        osmium::area::Assembler::config_type assembler_config;
        assembler_config.create_empty_areas = false;

        osmium::area::MultipolygonManager<osmium::area::Assembler> mp_manager{ assembler_config };
        osmium::relations::read_relations(input_file, mp_manager);

        index_t index;
        location_handler_t location_handler(index);
        osm_datahandler data_handler;
        
        osmium::io::Reader reader{ input_file, osmium::io::read_meta::no };
        osmium::apply(reader, location_handler, data_handler,
            mp_manager.handler([&data_handler](const osmium::memory::Buffer& area_buffer) {
                osmium::apply(area_buffer, data_handler);
            })
        );
        reader.close();

        out_data = data_handler.to_osmdata();
    }
    catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return false;
    }

    return true;
}