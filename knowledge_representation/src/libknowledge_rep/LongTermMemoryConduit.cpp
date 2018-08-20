#include <knowledge_representation/LongTermMemoryConduit.h>
#include <iostream>
#include <mysqlx/xdevapi.h>
#include <string>
#include <knowledge_representation/Concept.h>
#include <knowledge_representation/Instance.h>

using ::mysqlx::SessionSettings;
using ::mysqlx::Error;
using ::mysqlx::Session;
using ::mysqlx::Schema;
using ::mysqlx::Table;
using ::mysqlx::RowResult;
using ::mysqlx::Row;
using ::mysqlx::TableRemove;
using ::mysqlx::Result;
using std::vector;
using std::string;
using std::cout;
using std::endl;
using std::unique_ptr;

namespace knowledge_rep {

LongTermMemoryConduit::LongTermMemoryConduit(const string &addr, const uint port, const string &usr,
                                             const string &password, const string &db_name) {

  try {
    SessionSettings from_options(addr, port, usr, password, db_name);
    sess = std::unique_ptr<Session>(new Session(from_options));
    db = std::unique_ptr<Schema>(new Schema(*sess, db_name));
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

LongTermMemoryConduit::~LongTermMemoryConduit() = default;


bool LongTermMemoryConduit::add_entity(int id) {
  if (entity_exists(id)) {
    return false;
  }
  Table entities = db->getTable("entities");
  Result result = entities.insert("entity_id").values(id).execute();
  return true;
}

bool LongTermMemoryConduit::add_attribute(const string &name) {
  Table entities = db->getTable("attributes");
  Result result = entities.insert("attribute_name").values(name).execute();
  return true;
}

bool LongTermMemoryConduit::entity_exists(int id) const {
  Table entities = db->getTable("entities");
  auto result = entities.select("entity_id").where("entity_id = :id").bind("id", id).execute();
  return result.count() == 1;
}

bool LongTermMemoryConduit::entity_exists(const Entity& entity) const {
  Table entities = db->getTable("entities");
  auto result = entities.select("entity_id").where("entity_id = :id").bind("id", entity.entity_id).execute();
  return result.count() == 1;
}


bool LongTermMemoryConduit::remove_concept_references(const string &concept_name) {
  Concept concept = get_concept(concept_name);
  // Rely on the schema to clear out the childern via cascading delete
  concept.delete_entity();
  // Recreate it by getting it...
  Concept new_concept = get_concept(concept_name);
  return true;
}

vector<Entity> LongTermMemoryConduit::get_entities_with_attribute_of_value(const string &attribute_name,
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

vector<Entity> LongTermMemoryConduit::get_entities_with_attribute_of_value(const string &attribute_name,
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


vector<Entity> LongTermMemoryConduit::get_entities_with_attribute_of_value(const string &attribute_name,
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

std::vector<Entity> LongTermMemoryConduit::get_all_entities() {
  vector<Entity> all_obj_ids;
  Table entities = db->getTable("entities");
  RowResult rows = entities.select("*").execute();
  transform(rows.begin(), rows.end(), back_inserter(all_obj_ids), [this](Row row) {
    return Entity(row[0], *this);
  });
  return all_obj_ids;
}

void LongTermMemoryConduit::delete_all_entities() {
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

bool LongTermMemoryConduit::delete_attribute(int id) {
  // TODO: Write
  return false;
}

bool LongTermMemoryConduit::attribute_exists(int id) const {
  // TODO: Write this
  return false;
}

Concept LongTermMemoryConduit::get_concept(const string &name) {
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

Instance LongTermMemoryConduit::get_object_named(const string &name) {
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

bool LongTermMemoryConduit::get_entity(int entity_id, Entity &entity) {
  if (entity_exists(entity_id)) {
    entity = {entity_id, *this};
    return true;
  }
  return false;
}

Entity LongTermMemoryConduit::get_robot() {
  Entity robot = Entity(1, *this);
  assert(robot.is_valid());
  return robot;
}

/*
* Inserts a new entity into the database. Returns the entity's ID so
* it can be manipulated with other methods.
*/
Entity LongTermMemoryConduit::add_entity() {
  Table entities = db->getTable("entities");
  Result result = entities.insert("entity_id").values(NULL).execute();
  return {(int)result.getAutoIncrementValue(), *this};
}

std::vector<Concept> LongTermMemoryConduit::get_all_concepts() {
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

std::vector<Instance> LongTermMemoryConduit::get_all_instances() {
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

vector<std::pair<string, string> > LongTermMemoryConduit::get_all_attribute_names() const {
  vector<std::pair<string, string> > attribute_names;
  Table entities = db->getTable("attributes");
  RowResult rows = entities.select("*").execute();
  transform(rows.begin(), rows.end(), back_inserter(attribute_names), [this](Row row) {
      return std::make_pair(row[0], row[1]);
  });
  return attribute_names;
}


}

