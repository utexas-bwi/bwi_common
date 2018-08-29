#include <vector>
#include <knowledge_representation/LongTermMemoryConduit.h>
#include <knowledge_representation/Concept.h>
#include <knowledge_representation/Instance.h>
using std::vector;
using std::string;

namespace knowledge_rep {

vector<Instance> Concept::get_instances() const {
  auto entities = ltmc.get().get_entities_with_attribute_of_value("instance_of", entity_id);
  vector<Instance> instances;
  transform(entities.begin(), entities.end(), std::back_inserter(instances), [this](const Entity &entity){
    return Instance(entity.entity_id, ltmc.get());
  });
  return instances;
}

/**
 * @brief Recursively remove instances of the concept
 * An instance of a given concept "C" is transitively an instance of any concept that that defines an "is_a" relation
 * with "C". For example, if the concept of apple "is_a" concept of fruit, then removing instances of fruit will remove
 * all instances of apple.
 * @return the number of instances removed
 */
int Concept::remove_instances() {
  vector<Entity> child_concepts = ltmc.get().get_entities_with_attribute_of_value("is_a", entity_id);
  vector<Entity> instances_of_concept = ltmc.get().get_entities_with_attribute_of_value("instance_of", entity_id);
  
  int total_removed = 0;
  for (const auto &child : child_concepts) {
    total_removed += Concept(child.entity_id, ltmc).remove_instances();
  }
  for (auto &instance: instances_of_concept) {
    instance.delete_entity();
  }

  total_removed += instances_of_concept.size();
  return total_removed;
}

Instance Concept::create_instance() {
  auto instance = ltmc.get().add_entity();
  instance.add_attribute("instance_of", this->entity_id);
  return {instance.entity_id, ltmc.get()};
}

/**
 * @brief Create an instance of the concept and give it a particular name
 * @param name the name to give the instance
 * @return the created instance, or empty if the instance cannot be created (as when the name is taken)
 */
boost::optional<Instance> Concept::create_instance(const std::string& name) {
  auto instance = Instance(ltmc.get().add_entity().entity_id, ltmc);
  instance.add_attribute("instance_of", this->entity_id);
  bool added_name = instance.add_attribute("name", name);
  if (!added_name) {
    return {};
  }
  return instance;
}

/**
 * @brief Get all concepts that are direct children of this concept
 * 
 * @return all direct childern
 */
vector<Concept> Concept::get_children() const {
    // Is a should only apply to concepts
    auto entities = ltmc.get().get_entities_with_attribute_of_value("is_a", entity_id);
    vector<Concept> as_concept{};
    std::transform(entities.begin(), entities.end(), std::back_inserter(as_concept),
            [this](const Entity &entity){
      return Concept(entity.entity_id, ltmc);
    });
    return as_concept;
}

string Concept::get_name() {
  if (!name.empty()) {
    return name;
  }
  // There should only be one
  auto name_attrs = get_attributes("name");
  if (!name_attrs.empty()) {
    name = boost::get<string>(name_attrs[0].value);
    return name;
  }
  // Shouldn't be an unnamed concept
  assert(false);
}

/**
 * @brief Removes all references to a concept
 * References include all entity-attributes that refer to this concept. But note that this method doesn't
 * delete the entities that are the subjects of the references. This means that all instances of this
 * concept will continue to exist, but they will no longer be identified as instances of this concept.
 * @param concept_name 
 * @return 
 */
bool Concept::remove_references() {
  // Rely on the schema to clear out the childern via cascading delete
  delete_entity();
  {
    // Recreate it with the same ID
    auto new_entity = ltmc.get().get_entity(entity_id);
    if (!new_entity) {
      return false;
    }
  }
  // The current entity should still be valid (because we recreated it), so make it a concept again
  add_attribute("is_concept", true);
  return true;
}

}
