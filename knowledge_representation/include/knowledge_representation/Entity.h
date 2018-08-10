#ifndef KNOWLEDGE_REPRESENTATION_ENTITY_H
#define KNOWLEDGE_REPRESENTATION_ENTITY_H

#include <string>
#include <vector>
#include <knowledge_representation/LongTermMemoryConduit.h>
namespace knowledge_rep {

class EntityAttribute;

class Entity {

public:
  int entity_id;

  Entity(int entity_id, LongTermMemoryConduit &ltmc): entity_id(entity_id), ltmc(ltmc){}

/*  Entity(Entity &&that): entity_id(that.entity_id), ltmc(that.ltmc) {
  }

  constexpr Entity(const knowledge_rep::Entity& that):  entity_id(that.entity_id), ltmc(that.ltmc) {
  }*/

  bool add_attribute(const std::string &attribute_name, const std::string &string_val);

  bool add_attribute(const std::string &attribute_name, float float_val);

  bool add_attribute(const std::string &attribute_name, bool bool_val);

  bool add_attribute(const std::string &attribute_name, int other_entity_id);

  bool add_attribute(const std::string &attribute_name, const Entity &other_entity);

  bool remove_attribute(const std::string &attribute_name);

  std::vector<EntityAttribute> get_attributes() const;

  std::vector<EntityAttribute> get_attributes(const std::string &attribute_name) const;

    boost::optional<std::string> get_name() const;

  bool add_attribute(const std::string &attribute_name, const char string_val[]);
  
  bool make_instance_of(const Concept &concept);

  bool delete_entity();

  bool is_valid() const;

    bool operator==(const Entity &other) const {
    return this->entity_id == other.entity_id;
  }

  Entity& operator =(const Entity &that);

  std::vector<EntityAttribute> operator [](const std::string &attr_name) const {
    return get_attributes(attr_name);
  };


protected:
  std::reference_wrapper<LongTermMemoryConduit> ltmc;


};
}



#endif //KNOWLEDGE_REPRESENTATION_ENTITY_H
