
#include <osmium/index/map/flex_mem.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/area/assembler.hpp>
#include <osmium/area/multipolygon_manager.hpp>
#include <osmium/io/any_input.hpp>
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

// Osmium manager class that receives all the OSM data.
// 
// This duplicates some functionality from MultiPolygonManager 
// since CRTP makes it impossible to override way_not_in_any_relation etc.
//
class osm_data_manager : public osmium::relations::RelationsManager<osm_data_manager, true, true, true>
{
private:
    using Assembler = osmium::area::Assembler;

public:
    using AssemblerConfig = typename Assembler::config_type;

    explicit osm_data_manager(AssemblerConfig area_assembler_config) :
        m_assembler_config(std::move(area_assembler_config))
    {}

    // Called to evaluate which relations to keep
    bool new_relation(const osmium::Relation& relation) const
    {
        const char* type = relation.tags().get_value_by_key("type");
        if (type && (!std::strcmp(type, "multipolygon") || !std::strcmp(type, "boundary")))
        {
            auto& members = relation.members();
            return std::any_of(members.cbegin(), members.cend(), [](const osmium::RelationMember& member) {
                return member.type() == osmium::item_type::way;
            });
        }
        return false;
    }

    // Called when all relation props are known
    void complete_relation(const osmium::Relation& relation)
    {
        std::vector<const osmium::Way*> ways;
        ways.reserve(relation.members().size());

        for (const auto& member : relation.members()) {
            if (member.ref() != 0) {
                auto way = this->get_member_way(member.ref());
                // this fixes a bug in osmium
                if (!way) {
                    continue;
                }
                ways.push_back(way);
            }
        }
        try {
            Assembler assembler{ m_assembler_config };
            assembler(relation, ways, this->buffer());
        }
        catch (const osmium::invalid_location&) {
            // XXX ignore
        }
    }

    //void after_way(const osmium::Way& way)
    //{
    //    // you need at least 4 nodes to make up a polygon
    //    if (way.nodes().size() <= 3) {
    //        return;
    //    }
    //
    //    try {
    //        if (!way.nodes().front().location() || !way.nodes().back().location()) {
    //            throw osmium::invalid_location{ "invalid location" };
    //        }
    //        if (way.ends_have_same_location())
    //        {
    //            if (way.tags().has_tag("area", "no")) {
    //                return;
    //            }
    //            Assembler assembler{ m_assembler_config };
    //            assembler(way, this->buffer());
    //            this->possibly_flush();
    //        }
    //    }
    //    catch (const osmium::invalid_location&) {
    //        // XXX ignore
    //    }
    //}

    void way_not_in_any_relation(const osmium::Way& way)
    {
        auto& tags = way.tags();

        bool is_bldg_part;
        const char* highway = tags["highway"];
        
        if (highway && !str_equal(highway, "footway"))
        {
            int lanes, layer;
            double width;
            bool has_width = parse_num_if_exists(tags["width"], width);
            bool has_lanes = parse_num_if_exists(tags["lanes"], lanes);
            bool has_layer = parse_num_if_exists(tags["layer"], layer);

            if (!has_layer || (has_layer && layer >= 0)) // underground not supported yet
            {
                m_mesh_builder.add_highway({
                    .way = &way,
                    .type = WAY_TYPE_STREET,
                    .lanes = has_lanes ? lanes : -1,
                    .layer = has_layer ? layer : 0,
                    .width = has_width ? width : has_lanes ? lanes * 3.5 : 3.5
                });
            }
        }
        else if (str_equal(highway, "footway") && !str_equal(tags["footway"], "crossing"))
        {
            double width;
            bool has_width = parse_num_if_exists(tags["width"], width);

            m_mesh_builder.add_highway({
                .way = &way,
                .type = WAY_TYPE_FOOTWAY,
                .lanes = -1,
                .width = has_width ? width : 1.0
            });
        }
        else if (is_building_or_part(tags, is_bldg_part))
        {
            m_mesh_builder.add_building({
                .obj_type = OBJ_TYPE_WAY,
                .is_part = is_bldg_part,
                .way = &way
            });
        }
    }

    void area(const osmium::Area& area)
    {
        bool is_bldg_part;
        if (is_building_or_part(area.tags(), is_bldg_part)) 
        {
            m_mesh_builder.add_building({
                .obj_type = OBJ_TYPE_AREA,
                .is_part = is_bldg_part,
                .area = &area
            });
        }
    }

    osm_data build_meshes()
    {
        auto batch = m_mesh_builder.get_draw_data();

        osm_data ret;
        for (size_t i = 0; i < batch.size(); ++i)
        {
            auto& dd = batch[i];

            uint32_t verts_startidx = ret.data.num_verts();
            for (double v : dd.verts) {
                ret.data.verts.push_back(float(v));
            }
            uint32_t tri_startidx = ret.data.num_tris();
            for (uint tri_index : dd.tri_indices) {
                ret.data.tri_indices.push_back(tri_index + verts_startidx);
            }

            ret.color_ranges.push_back({
                tri_startidx,
                dd.num_tris(),
                dd.color
            });
        }
        return ret;
    }

private:
    bool is_building_or_part(const osmium::TagList& tags, bool& out_is_part) const
    {
        bool is_building = tags.has_key("building");
        bool is_building_part = str_equal(tags["building:part"], "yes");
        out_is_part = is_building_part;
        return is_building || is_building_part;
    }

private:
    AssemblerConfig m_assembler_config;
    mesh_builder m_mesh_builder; 
};

bool read_osmfile(const fs::path& path, osm_data& out_data)
{
    using index_t = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;
    using location_handler_t = osmium::handler::NodeLocationsForWays<index_t>;

    osmium::area::AssemblerConfig area_assembler_cfg;
    area_assembler_cfg.create_empty_areas = false;

    osm_data_manager data_manager{ area_assembler_cfg };

    try {
        const osmium::io::File input_file{ path.string() };

        osmium::relations::read_relations(input_file, data_manager);

        index_t index;
        location_handler_t location_handler(index);

        osmium::io::Reader reader{ input_file, osmium::io::read_meta::no };
        osmium::apply(reader, location_handler,
            data_manager.handler([&](const osmium::memory::Buffer& area_buffer) {
                osmium::apply(area_buffer, data_manager);
            })
        );
        reader.close();
    }
    catch (const std::exception& e) {
        logERROR("Error reading OSM file: %s", e.what());
        return false;
    }

    out_data = data_manager.build_meshes();
    return true;
}