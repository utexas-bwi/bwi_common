#include "LongTermMemoryConduitPostgreSQL.h"
#include <knowledge_representation/LongTermMemoryConduitInterface.h>
#include <iostream>
#include <string>
#include <knowledge_representation/LTMCConcept.h>
#include <knowledge_representation/LTMCInstance.h>

using std::string;
using std::vector;
namespace knowledge_rep {

typedef LTMCConcept<LongTermMemoryConduitPostgreSQL> Concept;
typedef LTMCInstance<LongTermMemoryConduitPostgreSQL> Instance;
typedef LTMCEntity<LongTermMemoryConduitPostgreSQL> Entity;

LongTermMemoryConduitPostgreSQL::LongTermMemoryConduitPostgreSQL(const string &db_name)
  : LongTermMemoryConduitInterface<LongTermMemoryConduitPostgreSQL>() {

  conn = std::unique_ptr<pqxx::connection>(
    new pqxx::connection{"postgresql://postgres:nopass@localhost/knowledge_base"});


}

LongTermMemoryConduitPostgreSQL::~LongTermMemoryConduitPostgreSQL() = default;

bool LongTermMemoryConduitPostgreSQL::add_entity(int id) {
  pqxx::work txn{*conn};
  pqxx::result result = txn.exec(
    "INSERT INTO entities VALUES (" + txn.quote(id) + ") ON CONFLICT DO NOTHING RETURNING entity_id");
  txn.commit();
  return result.size() == 1;
}

/**
 * @brief Add a new attribute
 * @param name the name of the attribute
 * @param allowed_types a bitmask representing the types allowed for the attribute
 * @return whether the attribute was added. Note that addition will fail if the attribute already exists.
 */
bool LongTermMemoryConduitPostgreSQL::add_new_attribute(const string &name, const std::string &type) {
  pqxx::work txn{*conn};
  pqxx::result result = txn.exec(
    "INSERT INTO attributes VALUES (" + txn.quote(name) + ", " + txn.quote(type) + ") ON CONFLICT DO NOTHING");
  txn.commit();
  return result.affected_rows() == 1;
}

/// \return true if an entity exists with the given ID
bool LongTermMemoryConduitPostgreSQL::entity_exists(uint id) const {
  pqxx::work txn{*conn};
  // Remove all entities
  auto result = txn.exec("SELECT 1 FROM entities WHERE entity_id=" + txn.quote(id));
  txn.commit();
  return result.size() == 1;
}


vector<Entity> LongTermMemoryConduitPostgreSQL::get_entities_with_attribute_of_value(const string &attribute_name,
                                                                                     const uint other_entity_id) {
  pqxx::work txn{*conn};
  // Remove all entities
  auto result = txn.exec("SELECT * FROM entity_attributes_id WHERE attribute_value=" + txn.quote(other_entity_id) +
                         " and attribute_name = " + txn.quote(attribute_name));
  txn.commit();

  vector<Entity> return_result;
  transform(result.begin(), result.end(), back_inserter(return_result), [this](const pqxx::tuple &row) {
    return Entity(row["attribute_value"].as<uint>(), *this);
  });
  return return_result;
}

vector<Entity> LongTermMemoryConduitPostgreSQL::get_entities_with_attribute_of_value(const string &attribute_name,
                                                                                     const bool bool_val) {
  assert(false);

}


vector<Entity> LongTermMemoryConduitPostgreSQL::get_entities_with_attribute_of_value(const string &attribute_name,
                                                                                     const string &string_val) {
  assert(false);

}

std::vector<Entity> LongTermMemoryConduitPostgreSQL::get_all_entities() {
  assert(false);

}

/**
 * @brief Remove all entities and all entity attributes except for the robot
 */
uint LongTermMemoryConduitPostgreSQL::delete_all_entities() {
  pqxx::work txn{*conn};

  // Remove all entities
  uint num_deleted = txn.exec("DELETE FROM entities").affected_rows();
  txn.commit();

  // Add the robot itself back
  add_entity(1);
  auto robot = Entity(1, *this);
  Concept robot_con = get_concept("robot");
  robot.add_attribute("instance_of", robot_con);
  assert(entity_exists(1));
  return num_deleted;
}

bool LongTermMemoryConduitPostgreSQL::delete_attribute(string &name) {
  // TODO: Write
  return false;
}

bool LongTermMemoryConduitPostgreSQL::attribute_exists(const string &name) const {
  pqxx::work txn{*conn};
  auto result = txn.exec("SELECT 1 FROM attributes WHERE attribute_name=" + txn.quote(name));
  txn.commit();
  return result.size() == 1;
}

/**
 * @brief Retrieves a concept of the given name, or creates one with the name if no such instance exists
 * @param name
 * @return the existing concept, or the newly created one. In either case, the instance will at least have the name
 *         passed as a parameter.
 */
Concept LongTermMemoryConduitPostgreSQL::get_concept(const string &name) {
  pqxx::work txn{*conn, "get_concept"};
  string query =
    "SELECT * FROM entity_attributes_str AS eas "
    "INNER JOIN entity_attributes_bool AS eab ON eas.entity_id = eab.entity_id "
    "WHERE eas.attribute_name = 'name' "
    "AND eas.attribute_value = " + txn.quote(name) + " "
                                                     "AND eab.attribute_name = 'is_concept' "
                                                     "AND eab.attribute_value = true";
  auto q_result = txn.exec(query);
  txn.commit();

  std::vector<EntityAttribute> result;

  if (result.empty()) {
    Entity new_concept = add_entity();
    new_concept.add_attribute("name", name);
    new_concept.add_attribute("is_concept", true);
    return {new_concept.entity_id, name, *this};
  } else {
    return {result[0].entity_id, name, *this};
  }
}

/**
 * @brief Retrieves an instance of the given name, or creates one with the name if no such instance exists
 * @param name
 * @return the existing instance, or the newly created one. In either case, the instance will at least have the name
 *         passed as a parameter.
 */
Instance LongTermMemoryConduitPostgreSQL::get_instance_named(const string &name) {
  pqxx::work txn{*conn, "get_instance_named"};
  string query =
    "SELECT * FROM entity_attributes_str AS eas "
    "LEFT JOIN entity_attributes_bool AS eab ON eas.entity_id = eab.entity_id "
    "WHERE eas.attribute_name = 'name' "
    "AND eas.attribute_value = " + txn.quote(name) + " "
                                                     "AND ((eab.attribute_name = 'is_concept' AND eab.attribute_value = false) "
                                                     "     OR (eab.entity_id is NULL))";
  auto q_result = txn.exec(query);
  txn.commit();
  std::vector<EntityAttribute> result;

  if (result.empty()) {
    Instance new_entity = Instance(add_entity().entity_id, *this);
    new_entity.add_attribute("name", name);
    new_entity.add_attribute("is_concept", false);
    return new_entity;
  } else {
    return {result[0].entity_id, *this};
  }
}

/**
 * @brief Returns an entity with the given ID, if it exists
 * @param entity_id the ID of the entity to fetch
 * @return the entity requested, or an empty optional if no such entity exists
 */
boost::optional<Entity> LongTermMemoryConduitPostgreSQL::get_entity(uint entity_id) {
  if (entity_exists(entity_id)) {
    return Entity{entity_id, *this};
  }
  return {};
}

/**
 * @brief Gets the instance representing the robot
 * @return an instance representing the robot the LTMC is running on
 */
Instance LongTermMemoryConduitPostgreSQL::get_robot() {
  Instance robot = Instance(1, *this);
  assert(robot.is_valid());
  return robot;
}

/**
 * @brief Inserts a new entity into the database. Returns the entity's ID so it can be manipulated with other methods.
 * @return the new entity
 */
Entity LongTermMemoryConduitPostgreSQL::add_entity() {
  pqxx::work txn{*conn, "add_entity"};
  // Remove all entities
  auto result = txn.exec("INSERT INTO entities VALUES (DEFAULT) RETURNING entity_id");
  txn.commit();
  return {result[0]["entity_id"].as<int>(), *this};
}

/**
 * @brief Queries for all entities that are marked as concepts
 * @return all concepts in the LTMC
 */
std::vector<Concept> LongTermMemoryConduitPostgreSQL::get_all_concepts() {
  vector<Concept> concepts;
  string query =
    "SELECT * FROM entity_attributes_bool "
    "WHERE attribute_name = 'is_concept' "
    "AND attribute_value = true ";
  std::vector<EntityAttribute> result;
  select_query<bool>(query, result);
  transform(result.begin(), result.end(), back_inserter(concepts),
            [this](EntityAttribute &attr) { return Concept(attr.entity_id, *this); });
  return concepts;
}

/**
 * @brief Queries for all entities that are identified as instances
 * @return all instances in the LTMC
 */
std::vector<Instance> LongTermMemoryConduitPostgreSQL::get_all_instances() {
  vector<Instance> concepts;
  string query =
    "SELECT * FROM entity_attributes_bool "
    "WHERE attribute_name = 'is_concept' "
    "AND attribute_value = false ";
  std::vector<EntityAttribute> result;
  select_query<bool>(query, result);
  transform(result.begin(), result.end(), back_inserter(concepts),
            [this](EntityAttribute &attr) { return Instance(attr.entity_id, *this); });
  return concepts;
}

/**
 * @brief Retrieves all attributes
 * @return a list of tuples. First element of each is the attribute name, the second is a bitmask representing acceptable
 * types for that attribute
 */
vector<std::pair<string, int> > LongTermMemoryConduitPostgreSQL::get_all_attributes() const {
  vector<std::pair<string, int> > attribute_names;
  assert(false);

  return attribute_names;
}


/**
 * @brief Deletes an entity and any other entities and relations that rely on it.
 * @return true if the entity was deleted. False if it could not be, or already was
 */
bool LongTermMemoryConduitPostgreSQL::delete_entity(Entity &entity) {

  // TODO: Handle failure
  // TODO: Recursively remove entities that are members of directional relations
  if (!entity.is_valid()) {
    return false;
  }
  // Because we've all references to this entity have foreign key relationships with cascade set,
  // this should clear out any references to this entity in other tables as well
  assert(false);

  // TODO: Actually check that we deleted something
  return true;
}

bool LongTermMemoryConduitPostgreSQL::add_attribute(Entity &entity, const std::string &attribute_name,
                                                    const float float_val) {
  add_new_attribute(attribute_name, "float");

  try {
    pqxx::work txn{*conn, "add_attribute (float)"};
    auto result = txn.exec(
      "INSERT INTO entity_attributes_float VALUES (" + txn.quote(entity.entity_id) + ", " + txn.quote(attribute_name) +
      ", " + txn.quote(float_val) + ")");
    txn.commit();
    return result.affected_rows() == 1;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return false;
  }

}


bool
LongTermMemoryConduitPostgreSQL::add_attribute(Entity &entity, const std::string &attribute_name, const bool bool_val) {
  add_new_attribute(attribute_name, "bool");

  try {
    pqxx::work txn{*conn, "add_attribute (bool)"};
    auto result = txn.exec(
      "INSERT INTO entity_attributes_bool VALUES (" + txn.quote(entity.entity_id) + ", " + txn.quote(attribute_name) +
      ", " + txn.quote(bool_val) + ")");
    txn.commit();
    return result.affected_rows() == 1;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return false;
  }

}


bool LongTermMemoryConduitPostgreSQL::add_attribute(Entity &entity, const std::string &attribute_name,
                                                    const uint other_entity_id) {
  add_new_attribute(attribute_name, "id");

  try {
    pqxx::work txn{*conn, "add_attribute (id)"};
    auto result = txn.exec(
      "INSERT INTO entity_attributes_id VALUES (" + txn.quote(entity.entity_id) + ", " + txn.quote(attribute_name) +
      ", " + txn.quote(other_entity_id) + ")");
    txn.commit();
    return result.affected_rows() == 1;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return false;
  }

}

bool LongTermMemoryConduitPostgreSQL::add_attribute(Entity &entity, const std::string &attribute_name,
                                                    const std::string &string_val) {
  add_new_attribute(attribute_name, "string");

  try {
    pqxx::work txn{*conn, "add_attribute (str)"};
    auto result = txn.exec(
      "INSERT INTO entity_attributes_str VALUES (" + txn.quote(entity.entity_id) + ", " + txn.quote(attribute_name) +
      ", " + txn.quote(string_val) + ")");
    txn.commit();
    return result.affected_rows() == 1;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return false;
  }

}

int LongTermMemoryConduitPostgreSQL::remove_attribute(Entity &entity, const std::string &attribute_name) {
  int removed_count = 0;
  for (const auto &table_name: table_names) {
    // TODO: Rewrite this as SQL query to delete from a join across attribute_name
    assert(false);

  }
  return removed_count;
}

int LongTermMemoryConduitPostgreSQL::remove_attribute_of_value(Entity &entity, const std::string &attribute_name,
                                                               const Entity &other_entity) {
  assert(false);
}


vector<EntityAttribute>
LongTermMemoryConduitPostgreSQL::get_attributes(const Entity &entity) const {
  assert(false);
}

std::vector<EntityAttribute>
LongTermMemoryConduitPostgreSQL::get_attributes(const Entity &entity, const std::string &attribute_name) const {
  assert(false);

}

/**
 * @brief Verifies that the entity still exists in the LTMC
 * An entity can be invalidated if it is explicitly deleted from the LTMC, or if
 * it is removed as a result of other helpers that delete entities.
 * @return whether the entity is valid
 */
bool LongTermMemoryConduitPostgreSQL::is_valid(const Entity &entity) const {
  return entity_exists(entity.entity_id);
}


/**
 * @brief Get all concepts that this instance is transitively an instance of
 * For example, if an entity A "instance_of" concept named apple, and apple "is_a" concept of fruit,
 * then get_concepts will return the concepts of both apple and fruit.
 * @return
 */
std::vector<Concept> LongTermMemoryConduitPostgreSQL::get_concepts(const Instance &instance) {
  assert(false);

}

}

