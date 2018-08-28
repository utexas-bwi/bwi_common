#include <utility>

#pragma once
#include <knowledge_representation/Entity.h>

namespace knowledge_rep {

class Concept : public Entity {
  std::string name;
public:

  Concept(int entity_id, std::string name, LongTermMemoryConduit &ltmc) : name(std::move(name)), Entity(entity_id, ltmc) {}

  Concept(int entity_id, LongTermMemoryConduit &ltmc) : Entity(entity_id, ltmc) {}

  Instance create_instance();

  boost::optional<Instance> create_instance(const std::string& name);

  int remove_instances();

  std::string get_name();

  std::vector<Instance> get_instances() const;

  std::vector<Concept> get_children() const;

  bool remove_references();

};
}


