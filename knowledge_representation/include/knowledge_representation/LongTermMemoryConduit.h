#pragma once
#include <iostream>
#include <string>
#include <map>
#include <boost/variant.hpp>
#include <boost/optional.hpp>
#include <utility>
#include <typeindex>
#include "EntityAttribute.h"

// Requirements
// * Be able to staticly select which database backend
// * No external dependency on which backend is available
// Options
// Non-requirements:
// * ABI compatibility. We will continue to assume source builds
//
// Solution: Static polymorphism w/ CRTP switched via a compile flag (?)
// * If we had used a pure virtual interface, we'd have to go through a pointer to the objects (LTMC, Entity, etc)
//   and we'd still have a vtable lookup
// The outside still sees
namespace knowledge_rep {

class Concept;
class Entity;

class Instance;

template <typename Impl>
class LongTermMemoryConduit {

  friend Entity;
  friend Concept;
  using LTMC = LongTermMemoryConduit;

public:

  LongTermMemoryConduit(const std::string &addr, uint port, const std::string &usr,
                        const std::string &password, const std::string &db_name);

  LongTermMemoryConduit(LongTermMemoryConduit &&that) = default;


  LongTermMemoryConduit& operator=(LongTermMemoryConduit &&that) noexcept {
      return static_cast<Impl*>(this)->operator=(that);
  }

  bool add_attribute(const std::string &name, int allowed_types){
      return static_cast<Impl*>(this)->add_attribute(name, allowed_types);
  };

  std::vector<Entity>
  get_entities_with_attribute_of_value(const std::string &attribute_name, const int other_entity_id){
      return static_cast<Impl*>(this)->get_entities_with_attribute_of_value(attribute_name, other_entity_id);
  };

  std::vector<Entity>
  get_entities_with_attribute_of_value(const std::string &attribute_name, const bool bool_val){
      static_cast<Impl*>(this)->get_entities_with_attribute_of_value(attribute_name, bool_val);
  }

  std::vector<Entity>
  get_entities_with_attribute_of_value(const std::string &attribute_name, const std::string &string_val) {
      static_cast<Impl*>(this)->get_entities_with_attribute_of_value(attribute_name, string_val);
  }

  bool entity_exists(int id) const {
      static_cast<Impl*>(this)->entity_exists(id);
  };

  bool delete_attribute(const std::string &name){
      static_cast<Impl*>(this)->delete_attribute(name);
  }

  bool attribute_exists(const std::string &name) const {
      static_cast<Impl*>(this)->attribute_exists(name);
  };

  void delete_all_entities(){
      static_cast<Impl*>(this)->delete_all_entities();
  }

  std::vector<Entity> get_all_entities(){
      static_cast<Impl*>(this)->get_all_entities();
  }

    std::vector<Concept> get_all_concepts() {
      return static_cast<Impl*>(this)->get_all_concepts();
  };

    std::vector<Instance> get_all_instances() {
        return static_cast<Impl*>(this)->get_all_instances();
    }

    std::vector<std::pair<std::string, int>> get_all_attributes() const{
        static_cast<Impl*>(this)->get_all_attributes();
    }



  bool select_query_int(const std::string &sql_query, std::vector<EntityAttribute> &result) const {
    return static_cast<Impl*>(this)->select_query_int(sql_query, result);
  }

  bool select_query_float(const std::string &sql_query, std::vector<EntityAttribute> &result) const {
    return static_cast<Impl*>(this)->select_query_float(sql_query, result);
  }

  bool select_query_string(const std::string &sql_query, std::vector<EntityAttribute> &result) const {
    return static_cast<Impl*>(this)->select_query_string(sql_query, result);
  }

  bool select_query_bool(const std::string &sql_query, std::vector<EntityAttribute> &result) const {
    return static_cast<Impl*>(this)->select_query_bool(sql_query, result);
  }


//// CONVENIENCE
  Concept get_concept(const std::string &name) {
        static_cast<Impl*>(this)->get_concept(name);
    };

  Instance get_instance_named(const std::string &name) {
      static_cast<Impl*>(this)->get_instance_named(name);
  };

  Instance get_robot() {
      static_cast<Impl*>(this)->get_robot();
  };

  Entity add_entity(){
      static_cast<Impl*>(this)->add_entity();
  };

  bool add_entity(int id){
      static_cast<Impl*>(this)->add_entity(id);
  };

  boost::optional<Entity> get_entity(int entity_id){
      static_cast<Impl*>(this)->get_entity(entity_id);

  };


};


