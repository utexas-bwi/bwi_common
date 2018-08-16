#ifndef KNOWLEDGE_REPRESENTATION_INSTANCE_H
#define KNOWLEDGE_REPRESENTATION_INSTANCE_H

#include <knowledge_representation/Entity.h>

namespace knowledge_rep {

class Instance : public Entity {
    std::string name;
public:

    Instance(int entity_id, const std::string &name, LongTermMemoryConduit &ltmc) : name(name),
                                                                                    Entity(entity_id, ltmc) {}

    Instance(int entity_id, LongTermMemoryConduit &ltmc) : Entity(entity_id, ltmc) {}

    std::string get_name();

    bool make_instance_of(const Concept &concept);

    std::vector<Concept> get_concepts();

};


}

#endif //KNOWLEDGE_REPRESENTATION_INSTANCE_H
