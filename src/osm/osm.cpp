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
#include <glm/glm.hpp>

#include "mesh_builder.hpp"
#include "osm.hpp"


// Reads all OSM data and adds it to the mesh builder.
// - All nodes are added before all ways/areas.
class osm_reader
{
public:
    explicit osm_reader(const std::string& filepath_or_url, mesh_builder& mesh_builder) :
        m_manager{ mesh_builder },
        m_file{ filepath_or_url }
    {}

    void read_all()
    {
        // First pass.
        osmium::relations::read_relations(m_file, m_manager);

        using loc_index_t = osmium::index::map::FlexMem<osmium::unsigned_object_id_type, osmium::Location>;
        using loc_handler_t = osmium::handler::NodeLocationsForWays<loc_index_t>;

        // note: invalid locations will throw an error
        loc_index_t loc_index;
        loc_handler_t loc_handler(loc_index);

        // Second pass.
        osmium::io::Reader reader{ m_file, osmium::io::read_meta::no };
        osmium::apply(reader, loc_handler, m_manager.handler([&](osmium::memory::Buffer&& buffer) {
            osmium::apply(buffer, m_manager); // objects in output buffer
        }));
        reader.close();
    }

private:
    class manager : public osmium::relations::RelationsManager<osm_reader, true, true, false, true>
    {
    public:
        using area_assembler = osmium::area::Assembler;
        using area_assembler_cfg = typename area_assembler::config_type;

    public:
        manager(mesh_builder& mesh_builder, area_assembler_cfg area_assm_cfg = {}) :
            m_mesh_builder(mesh_builder),
            m_area_assm_cfg(std::move(area_assm_cfg))
        {}

        // First pass.
        bool new_relation(const osmium::Relation& relation) const
        {
            const char* type = relation.tags()["type"];
            if (type && (!std::strcmp(type, "multipolygon") || !std::strcmp(type, "boundary")))
            {
                // Keep any mp relation with at least one member way.
                // Member relations will be discarded later (TRelations is false).
                auto& members = relation.members();
                return std::any_of(members.cbegin(), members.cend(), [](auto& member) {
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
                if (member.ref() != 0) {
                    // TNodes is true, might get nodes here.
                    if (member.type() == osmium::item_type::node) {
                        continue;
                    }
                    assert(member.type() == osmium::item_type::way);

                    auto* way = this->get_member_way(member.ref());
                    if (!way->tags().empty()) {
                        add_way_or_area(*way);
                    }
                    rel_ways.push_back(way);
                }
            }

            area_assembler assembler{ m_area_assm_cfg };
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

        // Second pass.
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
                area_assembler assembler{ m_area_assm_cfg };
                assembler(way, this->buffer());
            } 
            else {
                m_mesh_builder.add_way(way);
            }
        }

    private:
        area_assembler_cfg m_area_assm_cfg;
        mesh_builder& m_mesh_builder;
    };

private:
    manager m_manager;
    const osmium::io::File m_file;
};


bool read_osmfile(const std::string& filepath_or_url, osm_data& out_data)
{
    mesh_builder mesh_builder;
    try {
        osm_reader reader(filepath_or_url, mesh_builder);
        reader.read_all();
    }
    catch (const std::exception& e) {
        logERROR("Error reading OSM file: %s", e.what());
        return false;
    }

    mesh_builder.build();
    auto& batch = mesh_builder.draw_data();

    for (size_t i = 0; i < batch.size(); ++i)
    {
        auto& dd = batch[i];

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