#include <knowledge_representation/Entity.h>
#include <knowledge_representation/Concept.h>
#include <mysqlx/xdevapi.h>
#include <knowledge_representation/LongTermMemoryConduit.h>
#include <knowledge_representation/EntityAttribute.h>
#include <string>

using namespace ::mysqlx;
using namespace std;



namespace knowledge_rep {

Entity& Entity::operator =(const Entity &that) {
  this->entity_id = that.entity_id;
  this->ltmc = that.ltmc;
}

/*
 * Deletes an entity and any other entities and relations that rely on it.
 */
bool Entity::delete_entity() {

  // TODO: Handle failure
  // TODO: Recursively remove entities that are members of directional relations
  // First, removing all references to the entity

  // Because we've all references to this entity have foreign key relationships with cascade set,
  // this should clear out any references to this entity in other tables as well
  Table table = ltmc.get().db->getTable("entities");
  TableRemove remover = table.remove();
  remover.where("entity_id=:id").bind("id", entity_id);
  remover.execute();
}

bool Entity::add_attribute(const std::string &attribute_name,
                           const float float_val) {
  Table entity_attributes = ltmc.get().db->getTable("entity_attributes_float");
  TableInsert inserter = entity_attributes.insert("entity_id", "attribute_name", "attribute_value");
  inserter.values(entity_id, attribute_name, float_val);
  try {
    inserter.execute();
  }
  catch (const mysqlx::Error &err) {
    cout << "ERROR: " << err << endl;
    return false;
  }
  return true;
}


bool
Entity::add_attribute(const std::string &attribute_name, const bool bool_val) {
  Table entity_attributes = ltmc.get().db->getTable("entity_attributes_bool");
  TableInsert inserter = entity_attributes.insert("entity_id", "attribute_name", "attribute_value");
  inserter.values(entity_id, attribute_name, bool_val);
  try {
    inserter.execute();
  }
  catch (const mysqlx::Error &err) {
    cout << "Tried to add " << entity_id << " " << attribute_name << " " << bool_val << endl;
    cout << "ERROR: " << err << endl;
    return false;
  }
  return true;
}


bool Entity::add_attribute(const std::string &attribute_name,
                           const int other_entity_id) {
  Table entity_attributes = ltmc.get().db->getTable("entity_attributes_id");
  TableInsert inserter = entity_attributes.insert("entity_id", "attribute_name", "attribute_value");
  inserter.values(entity_id, attribute_name, other_entity_id);
  try {
    inserter.execute();
  }
  catch (const mysqlx::Error &err) {
    cout << "Tried to add " << entity_id << " " << attribute_name << " " << other_entity_id << endl;
    cout << "ERROR: " << err << endl;
    return false;
  }

  return true;
}

bool Entity::add_attribute(const std::string &attribute_name, const Entity &other_entity) {
  return add_attribute(attribute_name, other_entity.entity_id);
}

bool Entity::add_attribute(const std::string &attribute_name,
                           const std::string &string_val) {
  Table entity_attributes = ltmc.get().db->getTable("entity_attributes_str");
  TableInsert inserter = entity_attributes.insert("entity_id", "attribute_name", "attribute_value");
  inserter.values(entity_id, attribute_name, string_val);
  try {
    inserter.execute();
  }
  catch (const mysqlx::Error &err) {
    cout << "ERROR: " << err << endl;
    return false;
  }
}

bool Entity::add_attribute(const std::string &attribute_name,
                           const char string_val[]) {
  return add_attribute(attribute_name, std::string(string_val));
}

bool Entity::make_instance_of(const knowledge_rep::Concept &concept) {
    return add_attribute("instance_of", concept.entity_id);
}

bool Entity::remove_attribute(const std::string &attribute_name) {

  for (const auto &table_name: table_names) {
    // TODO: Rewrite this as SQL query to delete from a join across attribute_name
    Table entity_attributes = ltmc.get().db->getTable(table_name);
    TableRemove remover = entity_attributes.remove();
    remover.where("entity_id = :id and attribute_name = :name").bind("id", entity_id).bind("name",
                                                                                           attribute_name);
    remover.execute();
  }
}


vector<EntityAttribute>
Entity::get_attributes() const {
  vector<EntityAttribute> attributes;
  for (const auto &name: table_names) {
    Table entity_attributes = ltmc.get().db->getTable(name);
    RowResult result = entity_attributes.select("*").where("entity_id = :id").bind("id", entity_id).execute();
    auto result_rows = result.fetchAll();

    vector<EntityAttribute> table_attributes = ltmc.get().unwrap_attribute_rows(name, result_rows);
    attributes.insert(attributes.end(), table_attributes.begin(), table_attributes.end());
  }

  return attributes;
}

std::vector<EntityAttribute>
Entity::get_attributes(const std::string &attribute_name) const {
  vector<EntityAttribute> attributes;
  for (const auto &name: table_names) {
    Table entity_attributes = ltmc.get().db->getTable(name);
    RowResult result = entity_attributes.select("*").where("entity_id = :id and attribute_name = :attr").bind(
        "id",
        entity_id).bind(
        "attr", attribute_name).execute();

    auto result_rows = result.fetchAll();

    vector<EntityAttribute> table_attributes = ltmc.get().unwrap_attribute_rows(name, result_rows);
    attributes.insert(attributes.end(), table_attributes.begin(), table_attributes.end());
  }
  return attributes;
}

bool Entity::is_valid() const {
  return ltmc.get().entity_exists(entity_id);
}



}
