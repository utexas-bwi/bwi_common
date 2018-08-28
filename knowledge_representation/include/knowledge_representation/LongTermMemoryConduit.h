#pragma once
#include <iostream>
#include <mysqlx/xdevapi.h>
#include <string>
#include <map>
#include <boost/variant.hpp>
#include <boost/optional.hpp>
#include <utility>
#include <typeindex>
#include "EntityAttribute.h"

namespace knowledge_rep {

class Concept;
class Entity;

class Instance;

static const std::vector<std::string> table_names = {
    "entity_attributes_id",
    "entity_attributes_str",
    "entity_attributes_float",
    "entity_attributes_bool"};

class LongTermMemoryConduit {

  friend Entity;
  friend Concept;
  using LTMC = LongTermMemoryConduit;

public:

  std::unique_ptr<mysqlx::Session> sess;
  std::unique_ptr<mysqlx::Schema> db;

  LongTermMemoryConduit(const std::string &addr, uint port, const std::string &usr,
                        const std::string &password, const std::string &db_name);

  LongTermMemoryConduit(LongTermMemoryConduit &&that) = default;

  ~LongTermMemoryConduit();

  LongTermMemoryConduit& operator=(LongTermMemoryConduit &&that) noexcept {
    std::swap(sess, that.sess);
    std::swap(db, that.db);
  }

  bool add_attribute(const std::string &name, int allowed_types);

  std::vector<Entity>
  get_entities_with_attribute_of_value(const std::string &attribute_name, const int other_entity_id);

  std::vector<Entity>
  get_entities_with_attribute_of_value(const std::string &attribute_name, const bool bool_val);

  std::vector<Entity>
  get_entities_with_attribute_of_value(const std::string &attribute_name, const std::string &string_val);

  bool entity_exists(int id) const;

  bool delete_attribute(std::string name);

  bool attribute_exists(std::string name) const;

  void delete_all_entities();

  std::vector<Entity> get_all_entities();

    std::vector<Concept> get_all_concepts();

    std::vector<Instance> get_all_instances();

    std::vector<std::pair<std::string, int>> get_all_attributes() const;


  template<typename T, typename... Types>
  bool select_query_args(const std::string &sql_query, std::vector<EntityAttribute> &result,
                    Types &&... vals) const {
    try {
      auto sql_result = sess->sql(sql_query).bind(vals...).execute();
      result = unwrap_attribute_rows<T>(sql_result.fetchAll());
      return true;
    }
    catch (const mysqlx::Error &err) {
      std::cout << "ERROR: " << err << std::endl;
      return false;
    }

  }
  template <typename T>
  bool select_query(const std::string &sql_query, std::vector<EntityAttribute> &result) const {
    try {
      auto sql_result = sess->sql(sql_query).execute();
      result = unwrap_attribute_rows<T>(sql_result.fetchAll());
      return true;
    }
    catch (const mysqlx::Error &err) {
      std::cout << "ERROR: " << err << std::endl;
      return false;
    }

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
  Concept get_concept(const std::string &name);

  Instance get_instance_named(const std::string &name);

  Instance get_robot();

  Entity add_entity();

  bool add_entity(int id);

  boost::optional<Entity> get_entity(int entity_id);

private:
  template <typename T>
  std::vector<EntityAttribute> unwrap_attribute_rows(std::list<mysqlx::Row> rows) const {
    std::vector<EntityAttribute> result_map;
    for (auto &row: rows) {
      int obj_id = row[0];
      std::string attribute_name = row[1];
      T unwrapped = row[2];
      AttributeValue col_value(unwrapped);
      result_map.emplace_back(EntityAttribute(obj_id, attribute_name, col_value));
    }
    return result_map;
  }

  std::vector<EntityAttribute> unwrap_attribute_rows(const std::string &table_name, const std::list<mysqlx::Row> &rows) const {

    if (table_name == "entity_attributes_str") {
      return unwrap_attribute_rows<std::string>(rows);
    } else if (table_name == "entity_attributes_id") {
      return unwrap_attribute_rows<int>(rows);
    } else if (table_name == "entity_attributes_bool") {
      return unwrap_attribute_rows<bool>(rows);
    } else if (table_name == "entity_attributes_float") {
      return unwrap_attribute_rows<float>(rows);
    }
  }

};
template bool LongTermMemoryConduit::select_query<int>(const std::string &sql_query, std::vector<EntityAttribute> &result) const;

}


