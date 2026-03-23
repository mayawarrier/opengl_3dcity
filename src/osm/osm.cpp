//
// https://github.com/osmcode/libosmium/blob/master/examples/osmium_area_test.cpp
// https://github.com/osmcode/osmcode.github.io/blob/master/_libosmium_manual/12_working-with-relations.md
//

#include <osmium/index/map/flex_mem.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/area/assembler.hpp>
#include <osmium/relations/relations_manager.hpp>
#include <osmium/io/any_input.hpp> // todo: replace with pbf only later
#include <osmium/visitor.hpp>
#include <osmium/geom/coordinates.hpp>
#include <osmium/geom/mercator_projection.hpp>
#include <osmium/dynamic_handler.hpp>
#include <glm/glm.hpp>

#include "comp_builders/highways.hpp"
#include "comp_builders/buildings.hpp"
#include "mesh_builder.hpp"

#include "osm.hpp"


// Reads all OSM data and adds it to the mesh builder.
// - All nodes are added before all ways/areas.
class osm_reader
{
public:
    explicit osm_reader(const std::string& filepath_or_url, osmium::area::AssemblerConfig area_assm_cfg = {}) :
        m_file{ filepath_or_url },
        m_area_assm_cfg(area_assm_cfg)
    {}

    template <class TMeshBuilder>
    void read_all(TMeshBuilder& mesh_builder)
    {
        manager<TMeshBuilder> manager(*this, mesh_builder);

        // First pass.
        osmium::relations::read_relations(m_file, manager);

        using loc_index_t = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;
        using loc_handler_t = osmium::handler::NodeLocationsForWays<loc_index_t>;

        // note: invalid locations will throw an error
        loc_index_t loc_index;
        loc_handler_t loc_handler(loc_index);

        // Second pass.
        osmium::io::Reader reader{ m_file, osmium::io::read_meta::no };
        osmium::apply(reader, loc_handler, manager.handler([&](osmium::memory::Buffer&& buffer) {
            osmium::apply(buffer, manager);
        }));
        reader.close();
    }

private:
    template <class TMeshBuilder>
    class manager : public osmium::relations::RelationsManager<manager<TMeshBuilder>, true, true, false, true>
    {
    public:
        manager(osm_reader& reader, TMeshBuilder& mesh_builder) :
            m_mesh_builder(mesh_builder),
            m_reader(reader)
        {}

        // First pass.
        bool new_relation(const osmium::Relation& relation) const
        {
            if (is_multipolygon(relation.tags())) {
                return std::ranges::any_of(relation.members(), [](auto& member) {
                    return member.type() == osmium::item_type::way;
                });
            }
            return false;
        }

        // Second pass.
        void complete_relation(const osmium::Relation& relation)
        {
            std::vector<const osmium::Way*> rel_ways;
            rel_ways.reserve(relation.members().size());

            for (const auto& member : relation.members())
            {
                if (member.ref() != 0) 
                {
                    // TNodes is true, might get nodes. 
                    // Ignore them because they can't be part of a multipolygon.
                    if (member.type() == osmium::item_type::node) {
                        continue;
                    }
                    // should only have ways now!
                    assert(member.type() == osmium::item_type::way);

                    auto* way = this->get_member_way(member.ref());
                    if (!way->tags().empty()) {
                        add_way_or_area(*way);
                    }
                    rel_ways.push_back(way);
                }
            }
            osmium::area::Assembler assembler{ m_reader.m_area_assm_cfg };
            assembler(relation, rel_ways, this->buffer());
        }

        // Second pass.
        void after_node(const osmium::Node& node) {
            m_mesh_builder.add_node(node);
        }

        // Second pass.
        void way_not_in_any_relation(const osmium::Way& way) {
            add_way_or_area(way);
        }

        // Second pass (from buffer).
        void area(const osmium::Area& area) {
            m_mesh_builder.add_area(area);
        }

    private:
        void add_way_or_area(const osmium::Way& way)
        {
            assert_msg(
                way.nodes().front().location() && 
                way.nodes().back().location(), 
                "Location errors should have been thrown earlier");

            // is the way closed?
            if (way.nodes().size() >= 4 &&        // first and last node repeat 
                way.ends_have_same_location() &&  // ids don't match sometimes
                !way.tags().has_tag("area", "no")) 
            {
                osmium::area::Assembler assembler{ m_reader.m_area_assm_cfg };
                assembler(way, this->buffer());
            } 
            else {
                m_mesh_builder.add_way(way);
            }
        }

        bool is_multipolygon(const osmium::TagList& tags) const {
            const char* type = tags["type"];
            return type && (!std::strcmp(type, "multipolygon") || !std::strcmp(type, "boundary"));
        }

    private:
        TMeshBuilder& m_mesh_builder;
        osm_reader& m_reader;
    };

private:
    const osmium::io::File m_file;
    osmium::area::AssemblerConfig m_area_assm_cfg;
};


bool read_osmfile(const std::string& filepath_or_url, osm_data& out_data)
{
    osm_mesh_builder<
        bldg_comp_builder, 
        highway_comp_builder
    > mesh_builder;

    try {
        osm_reader reader(filepath_or_url);
        reader.read_all(mesh_builder);
    }
    catch (const std::exception& e) {
        logERROR("Error reading OSM file: %s", e.what());
        return false;
    }

    std::vector<draw_datad> drawdata;
    if (mesh_builder.build(drawdata)) 
    {
        for (size_t i = 0; i < drawdata.size(); ++i)
        {
            auto& dd = drawdata[i];

            uint32_t verts_startidx = out_data.data.num_verts();
            for (double v : dd.verts) {
                out_data.data.verts.push_back(float(v));
            }
            uint32_t tri_startidx = out_data.data.num_tris();
            for (uint tri_index : dd.tri_indices) {
                out_data.data.tri_indices.push_back(tri_index + verts_startidx);
            }

            out_data.color_ranges.push_back({
                tri_startidx,
                dd.num_tris(),
                dd.color
            });
        }
        return true;
    }

    return false;
}