
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

        mesh_builder::way_info wi{
            .id = way.id(),
            .name = tags["name"],
            .nodes = way.nodes()
        };

        const char* highway = tags["highway"];
        if (highway && !str_equal(highway, "footway")) 
        {    
            int lanes;
            double width;
            bool has_width = parse_num_if_exists(tags["width"], width);
            bool has_lanes = parse_num_if_exists(tags["lanes"], lanes);

            m_mesh_builder.add_highway({
                .way = std::move(wi),
                .type = WAY_TYPE_STREET,
                .lanes = has_lanes ? lanes : -1,
                .width = has_width ? width : has_lanes ? lanes * 3.5 : 3.5
            });
        }
        else if (str_equal(highway, "footway") && !str_equal(tags["footway"], "crossing"))
        {
            double width;
            bool has_width = parse_num_if_exists(tags["width"], width);

            m_mesh_builder.add_highway({
                .way = std::move(wi),
                .type = WAY_TYPE_FOOTWAY,
                .lanes = -1,
                .width = has_width ? width : 1.0
            });
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

                double ht_top = has_height ? height : (has_levels ? (3.0 * levels) : 3.0);
                double ht_btm = has_minheight ? min_height : (has_minlevel ? (3.0 * min_level) : 0.0);

                m_mesh_builder.add_building({
                    .way = std::move(wi),
                    .is_part = is_building_part,
                    .ht_btm = ht_btm,
                    .ht_top = ht_top
                });
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

    osm_data to_draw_data()
    {
        auto batch = m_mesh_builder.get_draw_data();

        osm_data ret;
        for (size_t i = 0; i < batch.size(); ++i) 
        {
            auto& dd = batch[i];
            auto dd_float = std::move(dd).as_float();

            uint32_t verts_startidx = ret.data.num_verts();
            ret.data.verts.insert(ret.data.verts.end(), dd_float.verts.begin(), dd_float.verts.end());

            uint32_t tri_startidx = ret.data.num_tris();
            for (uint tri_index : dd_float.tri_indices) {
                ret.data.tri_indices.push_back(tri_index + verts_startidx);
            }

            ret.color_ranges.push_back({
                tri_startidx,
                dd_float.num_tris(),
                dd_float.color
            });
        }
        return ret;
    }

private:
    mesh_builder m_mesh_builder;

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

        out_data = data_handler.to_draw_data();
    }
    catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return false;
    }

    return true;
}