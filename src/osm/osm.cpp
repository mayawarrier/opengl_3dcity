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
#include "comp_builders/water.hpp"
#include "mesh_builder.hpp"

#include "osm.hpp"


// Reads all OSM data and adds it to the mesh builder.
// - All nodes are added before all ways/areas.
class osm_reader
{
public:
    explicit osm_reader(const std::string& filepath_or_url, 
        const bbox2d& latlon_bounds, 
        osmium::area::AssemblerConfig area_assm_cfg = {}
    ) :
        m_file{ filepath_or_url },
        m_latlon_bounds(latlon_bounds),
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
            auto& tags = relation.tags();
            if (is_multipolygon(tags) && !is_underground(tags)) 
            {
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
                if (member.ref() != 0) {
                    // TNodes is true, might get nodes. 
                    // Ignore them because they can't be part of a multipolygon.
                    if (member.type() == osmium::item_type::node) {
                        continue;
                    }
                    // should only have ways now!
                    assert(member.type() == osmium::item_type::way);
                    rel_ways.push_back(this->get_member_way(member.ref()));
                }
            }
            osmium::area::Assembler assembler{ m_reader.m_area_assm_cfg };
            assembler(relation, rel_ways, this->buffer());
        }

        // Second pass.
        void after_node(const osmium::Node& node) {
            if (!loc_in_bounds(node.location())) return;
            m_mesh_builder.add_node(node);
        }

        // Second pass.
        void after_way(const osmium::Way& way) 
        {
            assert_msg(
                way.nodes().front().location() &&
                way.nodes().back().location(),
                "Location errors should have been thrown earlier");

            bool any_in_bounds = std::ranges::any_of(way.nodes(), [&](const auto& nr) {
                return loc_in_bounds(nr.location());
            });
            if (!any_in_bounds) {
                return;
            }

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

        // Second pass (from buffer).
        void area(const osmium::Area& area) 
        {
            bool any_in_bounds = false;
            for (const auto& outer_ring : area.outer_rings()) {
                for (const auto& nr : outer_ring) {
                    if (loc_in_bounds(nr.location())) {
                        any_in_bounds = true;
                        goto end;
                    }
                }
            }
            end:
            if (!any_in_bounds) { 
                return; 
            }

            m_mesh_builder.add_area(area);
        }

    private:
        bool loc_in_bounds(const osmium::Location& loc) {
            double lat = loc.lat();
            double lon = loc.lon();
            return m_reader.m_latlon_bounds.contains({ lon, lat });
        }

    private:
        TMeshBuilder& m_mesh_builder;
        osm_reader& m_reader;
    };

private:
    osmium::io::File m_file;
    bbox2d m_latlon_bounds;
    osmium::area::AssemblerConfig m_area_assm_cfg;
};


bool read_osmfile(const std::string& filepath_or_url, osm_gl_draw_data& out_data, const bbox2d& latlon_bounds)
{
    osm_mesh_builder<
        bldg_comp_builder,
        highway_comp_builder,
        water_comp_builder
    > mesh_builder;

    try {
        log_func("Reading OSM file", [&]() {
            osm_reader reader(filepath_or_url, latlon_bounds);
            reader.read_all(mesh_builder);
        });
    }
    catch (const std::exception& e) {
        logERROR("Error reading OSM file: %s", e.what());
        return false;
    }

    return mesh_builder.build(out_data);
}