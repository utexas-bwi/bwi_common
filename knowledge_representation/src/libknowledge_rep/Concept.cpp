#include <vector>
#include <knowledge_representation/LongTermMemoryConduit.h>
#include <knowledge_representation/Concept.h>
using std::vector;
using std::string;

namespace knowledge_rep {

vector<Entity> Concept::get_instances() const {
  return ltmc.get().get_entities_with_attribute_of_value("instance_of", entity_id);
}

bool Concept::remove_instances() {
  vector<Entity> child_concepts = ltmc.get().get_entities_with_attribute_of_value("is_a", entity_id);
  vector<Entity> instances_of_concept = ltmc.get().get_entities_with_attribute_of_value("instance_of", entity_id);
  for (const auto &child : child_concepts) {
    Concept(child.entity_id, ltmc).remove_instances();
  }
  for (auto &instance: instances_of_concept) {
    instance.delete_entity();
  }

  return true;
}

Entity Concept::create_instance() {
  auto instance = ltmc.get().add_entity();
  instance.add_attribute("instance_of", this->entity_id);
  return instance;
}

boost::optional<Entity> Concept::create_instance(const std::string& name) {
  auto instance = ltmc.get().add_entity();
  instance.add_attribute("instance_of", this->entity_id);
  bool added_name = instance.add_attribute("name", name);
  if (!added_name) {
    return {};
  }
  return instance;
}

vector<Concept> Concept::get_children() const {
    // Is a should only apply to concepts
    auto entities = ltmc.get().get_entities_with_attribute_of_value("is_a", entity_id);
    vector<Concept> as_concept;
    std::transform(entities.begin(), entities.end(), std::back_inserter(entities),
            [this](const Entity &entity){
      return Concept(entity_id, ltmc);
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
}

}