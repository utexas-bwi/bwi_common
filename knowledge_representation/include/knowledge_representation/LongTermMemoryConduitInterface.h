#pragma once

#include <iostream>
#include <string>
#include <map>
#include <boost/variant.hpp>
#include <boost/optional.hpp>
#include <utility>
#include <typeindex>
#include <vector>
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

template<typename ConLTMCImpl>
class LTMCConcept;

template<typename InsLTMCImpl>
class LTMCInstance;

template<typename EntLTMCImpl>
class LTMCEntity;

class EntityAttribute;


template<typename Impl>
class LongTermMemoryConduitInterface {
  using LTMC = LongTermMemoryConduitInterface;

public:

  using EntityImpl = LTMCEntity<Impl>;
  using InstanceImpl = LTMCInstance<Impl>;
  using ConceptImpl = LTMCConcept<Impl>;

  friend EntityImpl;
  friend InstanceImpl;
  friend ConceptImpl;
  friend Impl;

  LongTermMemoryConduitInterface(LongTermMemoryConduitInterface &&that) noexcept = default;


  LongTermMemoryConduitInterface &operator=(LongTermMemoryConduitInterface &&that) noexcept = default;

  bool add_attribute(const std::string &name, int allowed_types) {
    return static_cast<Impl *>(this)->add_attribute(name, allowed_types);
  };

  std::vector<EntityImpl>
  get_entities_with_attribute_of_value(const std::string &attribute_name, const int other_entity_id) {
    return static_cast<Impl *>(this)->get_entities_with_attribute_of_value(attribute_name, other_entity_id);
  };

  std::vector<EntityImpl>
  get_entities_with_attribute_of_value(const std::string &attribute_name, const bool bool_val) {
    return static_cast<Impl *>(this)->get_entities_with_attribute_of_value(attribute_name, bool_val);
  }

  std::vector<EntityImpl>
  get_entities_with_attribute_of_value(const std::string &attribute_name, const std::string &string_val) {
    return static_cast<Impl *>(this)->get_entities_with_attribute_of_value(attribute_name, string_val);
  }

  bool entity_exists(int id) const {
    return static_cast<const Impl *>(this)->entity_exists(id);
  };

  bool delete_attribute(const std::string &name) {
    return static_cast<Impl *>(this)->delete_attribute(name);
  }

  bool attribute_exists(const std::string &name) const {
    return static_cast<const Impl *>(this)->attribute_exists(name);
  };

  void delete_all_entities() {
    static_cast<Impl *>(this)->delete_all_entities();
  }

  std::vector<EntityImpl> get_all_entities() {
    return static_cast<Impl *>(this)->get_all_entities();
  }

  std::vector<ConceptImpl> get_all_concepts() {
    return static_cast<Impl *>(this)->get_all_concepts();
  };

  std::vector<InstanceImpl> get_all_instances() {
    return static_cast<Impl *>(this)->get_all_instances();
  }

  std::vector<std::pair<std::string, int>> get_all_attributes() const {
    return static_cast<const Impl *>(this)->get_all_attributes();
  }


  bool select_query_int(const std::string &sql_query, std::vector<EntityAttribute> &result) const {
    return static_cast<const Impl *>(this)->select_query_int(sql_query, result);
  }

  bool select_query_float(const std::string &sql_query, std::vector<EntityAttribute> &result) const {
    return static_cast<const Impl *>(this)->select_query_float(sql_query, result);
  }

  bool select_query_string(const std::string &sql_query, std::vector<EntityAttribute> &result) const {
    return static_cast<const Impl *>(this)->select_query_string(sql_query, result);
  }

  bool select_query_bool(const std::string &sql_query, std::vector<EntityAttribute> &result) const {
    return static_cast<const Impl *>(this)->select_query_bool(sql_query, result);
  }

//// CONVENIENCE
  ConceptImpl get_concept(const std::string &name) {
    return static_cast<Impl *>(this)->get_concept(name);
  };

  InstanceImpl get_instance_named(const std::string &name) {
    return static_cast<Impl *>(this)->get_instance_named(name);
  };

  InstanceImpl get_robot() {
    return static_cast<Impl *>(this)->get_robot();
  };

  EntityImpl add_entity() {
    return static_cast<Impl *>(this)->add_entity();
  };

  bool add_entity(int id) {
    return static_cast<Impl *>(this)->add_entity(id);
  };

  boost::optional<EntityImpl> get_entity(int entity_id) {
    return static_cast<Impl *>(this)->get_entity(entity_id);

  };

protected:
  bool delete_entity(EntityImpl &entity) {
    return static_cast<Impl *>(this)->delete_entity(entity);
  }

  bool add_attribute(EntityImpl &entity, const std::string &attribute_name,
                     const float float_val) {
    return static_cast<Impl *>(this)->add_attribute(entity, attribute_name, float_val);
  }


  bool add_attribute(EntityImpl &entity, const std::string &attribute_name, const bool bool_val) {
    return static_cast<Impl *>(this)->add_attribute(entity, attribute_name, bool_val);
  }

  bool add_attribute(EntityImpl &entity, const std::string &attribute_name,
                     const int other_entity_id) {
    return static_cast<Impl *>(this)->add_attribute(entity, attribute_name, other_entity_id);
  }

  bool add_attribute(EntityImpl &entity, const std::string &attribute_name,
                     const std::string &string_val) {
    return static_cast<Impl *>(this)->add_attribute(entity, attribute_name, string_val);
  }


  int remove_attribute(EntityImpl &entity, const std::string &attribute_name) {
    return static_cast<Impl *>(this)->remove_attribute(entity, attribute_name);
  }

  int remove_attribute_of_value(EntityImpl &entity, const std::string &attribute_name, const EntityImpl &other_entity) {
    return static_cast<Impl *>(this)->remove_attribute_of_value(entity, attribute_name, other_entity);
  }


  std::vector<EntityAttribute>
  get_attributes(const EntityImpl &entity) const {
    return static_cast<const Impl *>(this)->get_attributes(entity);
  }

  std::vector<EntityAttribute>
  get_attributes(const EntityImpl &entity, const std::string &attribute_name) const {
    return static_cast<const Impl *>(this)->get_attributes(entity, attribute_name);
  }

  bool is_valid(const EntityImpl &entity) const {
    return static_cast<const Impl *>(this)->is_valid(entity);
  }

  std::vector<ConceptImpl> get_concepts(const InstanceImpl &instance) {
    return static_cast<Impl *>(this)->get_concepts(instance);
  }

private:
  // We make the constructor private to make sure people can't build this interface type directly
  LongTermMemoryConduitInterface() {};
};


}
