#include <knowledge_representation/MemoryConduit.h>
#include <knowledge_representation/convenience.h>
#include <tf/transform_listener.h>
#include <bwi_perception/filter.h>
using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool knowledge_rep::MemoryConduit::encode(std::vector<PointCloudT::Ptr> &entities,
                                          PointCloudT::Ptr &table) {
    vector<EntityAttribute> facings = Entity(robot_id, ltmc).get_attributes("facing");
    assert(facings.size() == 1);
    int facing_id = boost::get<int>(facings.at(0).value);
    //TODO: Make sure this is table
    //assert(boost::get<std::string>(ltmc.get_entity_attribute(facing_id, "concept")) == string("table"));
    vector<Entity> candidates = ltmc.get_entities_with_attribute_of_value("is_on", facing_id);

    // TODO: Take the ones that do not have correspondences. Make new entities for them, put them on the table
    // TODO: Forget in STMC the old entities that weren't corresponded to
    //TODO: Change sensed in ltmc on the entities
    return false;
}


vector<knowledge_rep::EntityAttribute>
knowledge_rep::MemoryConduit::relevant_to(std::vector<int> entities) {
    vector<knowledge_rep::EntityAttribute> obj_attrs;
    // TODO: Implement a smarter strategy for getting relevant entity attributes
    // Something that will recurse and find dependencies, but also not get
    // everything in the database as a result

    // For now, use all entities
    for (const auto entity: ltmc.get_all_entities()) {
        auto attrs = entity.get_attributes();
        obj_attrs.insert(obj_attrs.end(), attrs.begin(), attrs.end());
    }
    return obj_attrs;
}

