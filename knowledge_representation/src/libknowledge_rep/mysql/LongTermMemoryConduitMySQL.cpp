#include "LongTermMemoryConduitMySQL.h"
#include <knowledge_representation/LongTermMemoryConduitInterface.h>
#include <iostream>
#include <mysqlx/xdevapi.h>
#include <string>
#include <knowledge_representation/LTMCConcept.h>
#include <knowledge_representation/LTMCInstance.h>
#include "LongTermMemoryConduitMySQL.h"

using ::mysqlx::SessionSettings;
using ::mysqlx::Error;
using ::mysqlx::Session;
using ::mysqlx::Schema;
using ::mysqlx::Table;
using ::mysqlx::TableInsert;
using ::mysqlx::RowResult;
using ::mysqlx::Row;
using ::mysqlx::TableRemove;
using ::mysqlx::Result;
using std::vector;
using std::string;
using std::cout;
using std::cerr;
using std::endl;
using std::unique_ptr;

namespace knowledge_rep {

typedef LTMCConcept<LongTermMemoryConduitMySQL> Concept;
typedef LTMCInstance<LongTermMemoryConduitMySQL> Instance;
typedef LTMCEntity<LongTermMemoryConduitMySQL> Entity;

LongTermMemoryConduitMySQL::LongTermMemoryConduitMySQL(const string &db_name="knowledge_base"): LongTermMemoryConduitInterface<LongTermMemoryConduitMySQL>() {

  std::cout << "In MySQL constructor" << std::endl;
  try {
    SessionSettings from_options("127.0.0.1", 33060, "root", "",  db_name);
    sess = std::unique_ptr<Session>(new Session(from_options));
    db = std::unique_ptr<Schema>(new Schema(*sess, db_name));
    cout << "DONE!" << endl;
  }
  catch (const mysqlx::Error &err) {
    cout << "ERROR: " << err << endl;
    return;
  }
  catch (std::exception &ex) {
    cout << "STD EXCEPTION: " << ex.what() << endl;
    return;
  }
  catch (const char *ex) {
    cout << "EXCEPTION: " << ex << endl;
    return;
  }

}

LongTermMemoryConduitMySQL::~LongTermMemoryConduitMySQL() = default;


bool LongTermMemoryConduitMySQL::add_entity(int id) {
  if (entity_exists(id)) {
    return false;
  }
  Table entities = db->getTable("entities");
  Result result = entities.insert("entity_id").values(id).execute();
  return true;
}

/**
 * @brief Add a new attribute
 * @param name the name of the attribute
 * @param allowed_types a bitmask representing the types allowed for the attribute
 * @return whether the attribute was added. Note that addition will fail if the attribute already exists.
 */
bool LongTermMemoryConduitMySQL::add_attribute(const string &name, const int allowed_types) {
  // TODO: Implement allowed_types parameter
  Table entities = db->getTable("attributes");
  Result result = entities.insert("attribute_name").values(name).execute();
  return true;
}

/// \return true if an entity exists with the given ID
bool LongTermMemoryConduitMySQL::entity_exists(int id) const {
  Table entities = db->getTable("entities");
  auto result = entities.select("entity_id").where("entity_id = :id").bind("id", id).execute();
  return result.count() == 1;
}


vector<Entity> LongTermMemoryConduitMySQL::get_entities_with_attribute_of_value(const string &attribute_name,
                                                                        const int other_entity_id) {
  Table entity_attributes = db->getTable("entity_attributes_id");
  RowResult result = entity_attributes.select("*").where(
      "attribute_value = :id and attribute_name = :attr").bind("id",
                                                               other_entity_id).bind(
      "attr", attribute_name).execute();
  std::list<Row> rows = result.fetchAll();

  vector<Entity> return_result;
  transform(rows.begin(), rows.end(), back_inserter(return_result), [this](Row row) mutable {
    return Entity((int)row[0], *this);
  });

  return return_result;
}

vector<Entity> LongTermMemoryConduitMySQL::get_entities_with_attribute_of_value(const string &attribute_name,
                                                                        const bool bool_val) {
  Table entity_attributes = db->getTable("entity_attributes_bool");
  RowResult result = entity_attributes.select("*").where(
      "attribute_value = :val and attribute_name = :attr").bind("val",
                                                                bool_val).bind(
      "attr", attribute_name).execute();
  std::list<Row> rows = result.fetchAll();
  vector<Entity> return_result;

  transform(rows.begin(), rows.end(), back_inserter(return_result), [this](Row row) {
    return Entity((int)(row[0]), *this);
  });

  return return_result;
}


vector<Entity> LongTermMemoryConduitMySQL::get_entities_with_attribute_of_value(const string &attribute_name,
                                                                        const string &string_val) {
  Table entity_attributes = db->getTable("entity_attributes_str");
  RowResult result = entity_attributes.select("*").where(
      "attribute_value = :val and attribute_name = :attr").bind("val",
                                                                string_val).bind(
      "attr", attribute_name).execute();
  std::list<Row> rows = result.fetchAll();
  vector<Entity> return_result;
  transform(rows.begin(), rows.end(), back_inserter(return_result), [this](Row row) {
    return Entity((int)row[0], *this);
  });

  return return_result;
}

std::vector<Entity> LongTermMemoryConduitMySQL::get_all_entities() {
  vector<Entity> all_obj_ids;
  Table entities = db->getTable("entities");
  RowResult rows = entities.select("*").execute();
  transform(rows.begin(), rows.end(), back_inserter(all_obj_ids), [this](Row row) {
    return Entity(row[0], *this);
  });
  return all_obj_ids;
}

/**
 * @brief Remove all entities and all entity attributes except for the robot
 */
void LongTermMemoryConduitMySQL::delete_all_entities() {
  Table table = db->getTable("entities");
  TableRemove remover = table.remove();
  remover.execute();

  // Add the robot itself back
  add_entity(1);
  auto robot = Entity(1, *this);
  Concept robot_con = get_concept("robot");
  robot.add_attribute("instance_of", robot_con);
  assert(entity_exists(1));
}

bool LongTermMemoryConduitMySQL::delete_attribute(string name) {
  // TODO: Write
  return false;
}

bool LongTermMemoryConduitMySQL::attribute_exists(string name) const {
  // TODO: Write this
  return false;
}

/**
 * @brief Retrieves a concept of the given name, or creates one with the name if no such instance exists
 * @param name
 * @return the existing concept, or the newly created one. In either case, the instance will at least have the name
 *         passed as a parameter.
 */
Concept LongTermMemoryConduitMySQL::get_concept(const string &name) {
  string query =
      "SELECT * FROM entity_attributes_str AS eas "
      "INNER JOIN entity_attributes_bool AS eab ON eas.entity_id = eab.entity_id "
      "WHERE eas.attribute_name = 'name' "
      "AND eas.attribute_value = ? "
      "AND eab.attribute_name = 'is_concept' "
      "AND eab.attribute_value = true";
  std::vector<EntityAttribute> result;
  select_query_args<string>(query, result, name);
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
Instance LongTermMemoryConduitMySQL::get_instance_named(const string &name) {
  string query =
      "SELECT * FROM entity_attributes_str AS eas "
      "LEFT JOIN entity_attributes_bool AS eab ON eas.entity_id = eab.entity_id "
      "WHERE eas.attribute_name = 'name' "
      "AND eas.attribute_value = ? "
      "AND ((eab.attribute_name = 'is_concept' AND eab.attribute_value = false) "
      "     OR (eab.entity_id is NULL))";
  std::vector<EntityAttribute> result;
  select_query_args<string>(query, result, name);
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
boost::optional<Entity> LongTermMemoryConduitMySQL::get_entity(int entity_id) {
  if (entity_exists(entity_id)) {
    return Entity{entity_id, *this};
  }
  return {};
}

/**
 * @brief Gets the instance representing the robot
 * @return an instance representing the robot the LTMC is running on
 */
Instance LongTermMemoryConduitMySQL::get_robot() {
  Instance robot = Instance(1, *this);
  assert(robot.is_valid());
  return robot;
}

/**
 * @brief Inserts a new entity into the database. Returns the entity's ID so it can be manipulated with other methods.
 * @return the new entity
 */
Entity LongTermMemoryConduitMySQL::add_entity() {
  Table entities = db->getTable("entities");
  Result result = entities.insert("entity_id").values(NULL).execute();
  return {(int)result.getAutoIncrementValue(), *this};
}

/**
 * @brief Queries for all entities that are marked as concepts
 * @return all concepts in the LTMC
 */
std::vector<Concept> LongTermMemoryConduitMySQL::get_all_concepts() {
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
std::vector<Instance> LongTermMemoryConduitMySQL::get_all_instances() {
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
vector<std::pair<string, int> > LongTermMemoryConduitMySQL::get_all_attributes() const {
  vector<std::pair<string, int> > attribute_names;
  Table entities = db->getTable("attributes");
  RowResult rows = entities.select("*").execute();
  transform(rows.begin(), rows.end(), back_inserter(attribute_names), [this](Row row) {
      return std::make_pair(row[0], row[1]);
  });
  return attribute_names;
}




/**
 * @brief Deletes an entity and any other entities and relations that rely on it.
 * @return true if the entity was deleted. False if it could not be, or already was
 */
  bool LongTermMemoryConduitMySQL::delete_entity(Entity &entity) {

    // TODO: Handle failure
    // TODO: Recursively remove entities that are members of directional relations
    if (!entity.is_valid()) {
      return false;
    }
    // Because we've all references to this entity have foreign key relationships with cascade set,
    // this should clear out any references to this entity in other tables as well
    Table table = db->getTable("entities");
    TableRemove remover = table.remove();
    remover.where("entity_id=:id").bind("id", entity.entity_id);
    remover.execute();
    // TODO: Actually check that we deleted something
    return true;
  }

  bool LongTermMemoryConduitMySQL::add_attribute(Entity &entity, const std::string &attribute_name,
                                                         const float float_val) {
    Table entity_attributes = db->getTable("entity_attributes_float");
    TableInsert inserter = entity_attributes.insert("entity_id", "attribute_name", "attribute_value");
    inserter.values(entity.entity_id, attribute_name, float_val);
    try {
      inserter.execute();
    }
    catch (const mysqlx::Error &err) {
      cerr << "Tried to add attribute " << attribute_name << " with value " << float_val << endl;
      cerr << "ERROR: " << err << endl;
      return false;
    }
    return true;
  }


  bool LongTermMemoryConduitMySQL::add_attribute(Entity &entity, const std::string &attribute_name, const bool bool_val) {
    Table entity_attributes = db->getTable("entity_attributes_bool");
    TableInsert inserter = entity_attributes.insert("entity_id", "attribute_name", "attribute_value");
    inserter.values(entity.entity_id, attribute_name, bool_val);
    try {
      inserter.execute();
    }
    catch (const mysqlx::Error &err) {
      cerr << "Tried to add " << entity.entity_id << " " << attribute_name << " " << bool_val << endl;
      cerr << "ERROR: " << err << endl;
      return false;
    }
    return true;
  }


  bool LongTermMemoryConduitMySQL::add_attribute(Entity &entity, const std::string &attribute_name,
                                                         const int other_entity_id) {
    Table entity_attributes = db->getTable("entity_attributes_id");
    TableInsert inserter = entity_attributes.insert("entity_id", "attribute_name", "attribute_value");
    inserter.values(entity.entity_id, attribute_name, other_entity_id);
    try {
      inserter.execute();
    }
    catch (const mysqlx::Error &err) {
      cerr << "Tried to add attribute " << attribute_name << " with value " << other_entity_id << endl;
      cerr << "ERROR: " << err << endl;
      return false;
    }

    return true;
  }

  bool LongTermMemoryConduitMySQL::add_attribute(Entity &entity, const std::string &attribute_name,
                                                         const std::string &string_val) {
    Table entity_attributes = db->getTable("entity_attributes_str");
    TableInsert inserter = entity_attributes.insert("entity_id", "attribute_name", "attribute_value");
    inserter.values(entity.entity_id, attribute_name, string_val);
    try {
      inserter.execute();
    }
    catch (const mysqlx::Error &err) {
      cerr << "Tried to add attribute " << attribute_name << " with value " << string_val << endl;
      cerr << "Message: " << err << endl;
      return false;
    }
    return true;
  }

  int LongTermMemoryConduitMySQL::remove_attribute(Entity &entity, const std::string &attribute_name) {
    int removed_count = 0;
    for (const auto &table_name: table_names) {
      // TODO: Rewrite this as SQL query to delete from a join across attribute_name
      Table entity_attributes = db->getTable(table_name);
      TableRemove remover = entity_attributes.remove();
      remover.where("entity_id = :id and attribute_name = :name").bind("id", entity.entity_id).bind("name",
                                                                                             attribute_name);
      auto result = remover.execute();
      removed_count += result.getAffectedItemsCount();

    }
    return removed_count;
  }

  int LongTermMemoryConduitMySQL::remove_attribute_of_value(Entity &entity, const std::string &attribute_name, const Entity &other_entity) {

    Table entity_attributes = db->getTable("entity_attributes_id");
    TableRemove remover = entity_attributes.remove();
    remover.where("entity_id = :id and attribute_name = :name and attribute_value = :value").bind("id", entity.entity_id).bind("name",
                                                                                                                        attribute_name).bind("value", other_entity.entity_id);
    auto result = remover.execute();
    return result.getAffectedItemsCount();
  }


  vector<EntityAttribute>
  LongTermMemoryConduitMySQL::get_attributes(const Entity &entity) const {
    vector<EntityAttribute> attributes;
    for (const auto &name: table_names) {
      Table entity_attributes = db->getTable(name);
      RowResult result = entity_attributes.select("*").where("entity_id = :id").bind("id", entity.entity_id).execute();
      auto result_rows = result.fetchAll();

      vector<EntityAttribute> table_attributes = unwrap_attribute_rows(name, result_rows);
      attributes.insert(attributes.end(), table_attributes.begin(), table_attributes.end());
    }

    return attributes;
  }

  std::vector<EntityAttribute>
  LongTermMemoryConduitMySQL::get_attributes(const Entity &entity, const std::string &attribute_name) const {
    vector<EntityAttribute> attributes;
    for (const auto &name: table_names) {
      Table entity_attributes = db->getTable(name);
      RowResult result = entity_attributes.select("*").where("entity_id = :id and attribute_name = :attr").bind(
          "id",
          entity.entity_id).bind(
          "attr", attribute_name).execute();

      auto result_rows = result.fetchAll();

      vector<EntityAttribute> table_attributes = unwrap_attribute_rows(name, result_rows);
      attributes.insert(attributes.end(), table_attributes.begin(), table_attributes.end());
    }
    return attributes;
  }

/**
 * @brief Verifies that the entity still exists in the LTMC
 * An entity can be invalidated if it is explicitly deleted from the LTMC, or if
 * it is removed as a result of other helpers that delete entities.
 * @return whether the entity is valid
 */
  bool LongTermMemoryConduitMySQL::is_valid(const Entity &entity) const {
    return entity_exists(entity.entity_id);
  }


/**
 * @brief Get all concepts that this instance is transitively an instance of
 * For example, if an entity A "instance_of" concept named apple, and apple "is_a" concept of fruit,
 * then get_concepts will return the concepts of both apple and fruit.
 * @return
 */
std::vector<Concept> LongTermMemoryConduitMySQL::get_concepts(const Instance &instance){
 auto results = sess->sql("CALL get_concepts(?)").bind(instance.entity_id).execute();
 auto rows = results.fetchAll();
 std::vector<Concept> concepts{};
 std::transform(rows.begin(), rows.end(), std::back_inserter(concepts), [this](const mysqlx::Row &row) {
     return Concept(row[0], *this);
 });
 return concepts;
}

}

