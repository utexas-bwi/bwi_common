#pragma once
#include <utility>

#include <knowledge_representation/Entity.h>

namespace knowledge_rep {

class Instance : public Entity {
    std::string name;
public:

    Instance(int entity_id, std::string name, LongTermMemoryConduit &ltmc) : name(std::move(name)),
                                                                                    Entity(entity_id, ltmc) {}

    Instance(int entity_id, LongTermMemoryConduit &ltmc) : Entity(entity_id, ltmc) {}

    boost::optional<std::string> get_name();

    bool make_instance_of(const Concept &concept);

    std::vector<Concept> get_concepts();

    bool has_concept(const Concept &concept);

};


}

