#ifndef KNOWLEDGE_REPRESENTATION_CONCEPT_H
#define KNOWLEDGE_REPRESENTATION_CONCEPT_H

#include <knowledge_representation/Entity.h>

namespace knowledge_rep {

class Concept : public Entity {
  std::string name;
public:


  Concept(int entity_id, const std::string &name, LongTermMemoryConduit &ltmc) : name(name), Entity(entity_id, ltmc) {}

  Concept(int entity_id, LongTermMemoryConduit &ltmc) : Entity(entity_id, ltmc) {}

  Entity create_instance();

  boost::optional<Entity> create_instance(const std::string& name);

  bool remove_instances();

  std::string get_name();

  std::vector<Entity> get_instances() const;

  std::vector<Concept> get_children() const;

};
}


#endif //KNOWLEDGE_REPRESENTATION_CONCEPT_H
