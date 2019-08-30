#pragma once
#include <knowledge_representation/LongTermMemoryConduitInterface.h>
#include <pqxx/pqxx>

namespace knowledge_rep {
static const char *table_names[] = {
  "entity_attributes_id",
  "entity_attributes_str",
  "entity_attributes_bool",
  "entity_attributes_float"};


class LongTermMemoryConduitPostgreSQL : public LongTermMemoryConduitInterface<LongTermMemoryConduitPostgreSQL> {

  using EntityImpl = LTMCEntity<LongTermMemoryConduitPostgreSQL>;
  using InstanceImpl = LTMCInstance<LongTermMemoryConduitPostgreSQL>;
  using ConceptImpl = LTMCConcept<LongTermMemoryConduitPostgreSQL>;

  friend EntityImpl;
  friend InstanceImpl;
  friend ConceptImpl;

  friend class LongTermMemoryConduitInterface;

public:

  std::unique_ptr<pqxx::connection> conn;

  explicit LongTermMemoryConduitPostgreSQL(const std::string &db_name);

  // Move constructor
  LongTermMemoryConduitPostgreSQL(LongTermMemoryConduitPostgreSQL &&that) = default;

  ~LongTermMemoryConduitPostgreSQL();

  // Move assignment
  LongTermMemoryConduitPostgreSQL &operator=(LongTermMemoryConduitPostgreSQL &&that) noexcept = default;

  bool add_new_attribute(const std::string &name, const AttributeValueType type);

  std::vector<EntityImpl>
  get_entities_with_attribute_of_value(const std::string &attribute_name, const uint other_entity_id);

  std::vector<EntityImpl>
  get_entities_with_attribute_of_value(const std::string &attribute_name, const bool bool_val);

  std::vector<EntityImpl>
  get_entities_with_attribute_of_value(const std::string &attribute_name, const std::string &string_val);

  bool entity_exists(uint id) const;

  bool delete_attribute(std::string &name);

  bool attribute_exists(const std::string &name) const;

  uint delete_all_entities();

  uint delete_all_attributes();

  std::vector<EntityImpl> get_all_entities();

  std::vector<ConceptImpl> get_all_concepts();

  std::vector<InstanceImpl> get_all_instances();

  std::vector<std::pair<std::string, AttributeValueType>> get_all_attributes() const;

  template<typename T>
  bool select_query(const std::string &sql_query, std::vector<EntityAttribute> &result) const {
    try {
      pqxx::work txn{*conn};
      auto query_result = txn.exec(sql_query);
      for (const auto &row: query_result) {
        result.emplace_back(row["entity_id"].as<uint>(), row["attribute_name"].as<std::string>(),
                            row["attribute_name"].as<T>());
      }
    } catch (const std::exception &e) {
      std::cerr << e.what() << std::endl;
      return false;
    }
    return true;
  }

  bool select_query_int(const std::string &sql_query, std::vector<EntityAttribute> &result) const {
    return select_query<int>(sql_query, result);
  }

  bool select_query_float(const std::string &sql_query, std::vector<EntityAttribute> &result) const {
    return select_query<float>(sql_query, result);
  }

  bool select_query_string(const std::string &sql_query, std::vector<EntityAttribute> &result) const {
    return select_query<std::string>(sql_query, result);
  }

  bool select_query_bool(const std::string &sql_query, std::vector<EntityAttribute> &result) const {
    return select_query<bool>(sql_query, result);
  }

// TODO: Implement this
// TODO: Move this to a higher level location.
// This interface supports general purpose reasoning over the knowledge base
// Takes in some representation of constraints, outputs the parsed answer set
// bool asp_query(std::string &query, std::vector<std::string> result) {
//	// Dump knowledge
//
//	// Use actasp to query
//}
//
//bwi_door(D) :- door(D), has(bwi, P), has(P, R), has(R, D).
//bwi_door(D) :- door(D), has(bwi, R), has(R, D).
//
//#show bwi_door/1.
//
//#
//#bwi_door(d3_414b1). bwi_door(d3_414b2


//// CONVENIENCE
  LTMCConcept<LongTermMemoryConduitPostgreSQL> get_concept(const std::string &name);

  InstanceImpl get_instance_named(const std::string &name);

  InstanceImpl get_robot();

  EntityImpl add_entity();

  bool add_entity(int id);

  boost::optional<EntityImpl> get_entity(uint entity_id);

protected:

  bool delete_entity(EntityImpl &entity);

  bool add_attribute(EntityImpl &entity, const std::string &attribute_name,
                     const float float_val);


  bool
  add_attribute(EntityImpl &entity, const std::string &attribute_name, const bool bool_val);

  bool add_attribute(EntityImpl &entity, const std::string &attribute_name,
                     const uint other_entity_id);

  bool add_attribute(EntityImpl &entity, const std::string &attribute_name,
                     const std::string &string_val);

  int remove_attribute(EntityImpl &entity, const std::string &attribute_name);

  int remove_attribute_of_value(EntityImpl &entity, const std::string &attribute_name,
                                const EntityImpl &other_entity);


  std::vector<EntityAttribute>
  get_attributes(const EntityImpl &entity) const;

  std::vector<EntityAttribute>
  get_attributes(const EntityImpl &entity, const std::string &attribute_name) const;

  bool is_valid(const EntityImpl &entity) const;

  std::vector<ConceptImpl> get_concepts(const InstanceImpl &instance);

};

typedef LTMCEntity<LongTermMemoryConduitPostgreSQL> Entity;
typedef LTMCConcept<LongTermMemoryConduitPostgreSQL> Concept;
typedef LTMCInstance<LongTermMemoryConduitPostgreSQL> Instance;
typedef LongTermMemoryConduitPostgreSQL LongTermMemoryConduit;
}
