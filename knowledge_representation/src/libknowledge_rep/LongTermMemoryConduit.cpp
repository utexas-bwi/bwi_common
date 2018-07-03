#include <knowledge_representation/LongTermMemoryConduit.h>
#include <iostream>
#include <mysqlx/xdevapi.h>
#include <string>
using namespace ::mysqlx;
using namespace std;

namespace knowledge_rep {

    const std::string LongTermMemoryConduit::table_names[] = {"entity_attributes_id" ,"entity_attributes_str" , "entity_attributes_float" , "entity_attributes_bool"};


    LongTermMemoryConduit::LongTermMemoryConduit(const std::string &addr, const uint port, const std::string &usr,
												 const std::string &password, const std::string &db_name) {

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

    /*
     * Inserts a new entity into the database. Returns the entity's ID so
     * it can be manipulated with other methods.
     */
    int LongTermMemoryConduit::add_entity() {
        Table entities = db->getTable("entities");
        Result result = entities.insert("entity_id").values(NULL).execute();
        return result.getAutoIncrementValue();
    }

    bool LongTermMemoryConduit::add_entity(int id) {
        if (entity_exists(id)) {
            return false;
        }
        Table entities = db->getTable("entities");
        Result result = entities.insert("entity_id").values(id).execute();
        return true;
    }

    bool LongTermMemoryConduit::add_attribute(const std::string &name) {
        Table entities = db->getTable("attributes");
        Result result = entities.insert("attribute_name").values(name).execute();
        return true;
    }

    /*
     * Deletes an entity and any other entities and relations that rely on it.
     */
    bool LongTermMemoryConduit::delete_entity(int id) {

        // TODO: Handle failure
        // TODO: Recursively remove entities that are members of directional relations
        // First, removing all references to the entity

        // Because we've all references to this entity have foreign key relationships with cascade set,
        // this should clear out any references to this entity in other tables as well
        Table table = db->getTable("entities");
        TableRemove remover = table.remove();
        remover.where("entity_id=:id").bind("id", id);
        remover.execute();
    }

	bool LongTermMemoryConduit::entity_exists(int id) {
		Table entities = db->getTable("entities");
		auto result = entities.select("entity_id").where("entity_id = :id").bind("id", id).execute();
		return result.count() == 1;
	}

    bool LongTermMemoryConduit::add_entity_attribute(int entity_id, const std::string &attribute_name,
                                                     const float float_val) {
        Table entity_attributes = db->getTable("entity_attributes_float");
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
    LongTermMemoryConduit::add_entity_attribute(int entity_id, const std::string &attribute_name, const bool bool_val) {
        Table entity_attributes = db->getTable("entity_attributes_bool");
        TableInsert inserter = entity_attributes.insert("entity_id", "attribute_name", "attribute_value");
        inserter.values(entity_id, attribute_name, bool_val);
        try {
            inserter.execute();
        }
        catch (const mysqlx::Error &err) {
            cout << "Tried to add " << entity_id << " " << attribute_name << " " << bool_val;
            cout << "ERROR: " << err << endl;
            return false;
        }
        return true;
    }


    bool LongTermMemoryConduit::add_entity_attribute(int entity_id, const std::string &attribute_name,
                                                     const int other_entity_id) {
        Table entity_attributes = db->getTable("entity_attributes_id");
        TableInsert inserter = entity_attributes.insert("entity_id", "attribute_name", "attribute_value");
        inserter.values(entity_id, attribute_name, other_entity_id);
        try {
            inserter.execute();
        }
        catch (const mysqlx::Error &err) {
            cout << "Tried to add " << entity_id << " " << attribute_name << " " << other_entity_id;
            cout << "ERROR: " << err << endl;
            return false;
        }


        return true;
    }

    bool LongTermMemoryConduit::add_entity_attribute(int entity_id, const std::string &attribute_name, const std::string &string_val) {
        Table entity_attributes = db->getTable("entity_attributes_str");
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

    bool LongTermMemoryConduit::add_entity_attribute(int entity_id, const std::string &attribute_name,
                                                     const char string_val[]) {
        return add_entity_attribute(entity_id, attribute_name, std::string(string_val));
    }

    bool LongTermMemoryConduit::remove_entity_attribute(int entity_id, const std::string &attribute_name) {

        for (const auto & table_name: table_names) {
            Table entity_attributes = db->getTable(table_name);
            TableRemove remover = entity_attributes.remove();
            remover.where("entity_id = :id and attribute_name = :name").bind("id", entity_id).bind("name",
                                                                                                   attribute_name);
            remover.execute();
        }
    }


    bool LongTermMemoryConduit::remove_concept_references(const std::string &concept_name) {
        int concept_id = get_concept(concept_name);
        delete_entity(concept_id);
        int new_concept_id = add_entity();
        add_entity_attribute(new_concept_id, "concept", concept_name);

        return true;
    }

    bool LongTermMemoryConduit::remove_children_of_entity(int entity_id) {
        vector<int> children = get_entities_with_attribute_of_value("is_a", entity_id);
        for (auto& child : children) {
            remove_children_of_entity(child);
            delete_entity(child);
        }

        return true;
    }


    bool LongTermMemoryConduit::remove_entities_of_concept(const std::string &concept_name) {
        int concept_id = get_concept(concept_name);
        remove_children_of_entity(concept_id);

        return true;
    }


    LongTermMemoryConduit::ConceptValue LongTermMemoryConduit::unwrap_attribute_row_value(mysqlx::Value wrapped) const {
        switch (wrapped.getType()) {
            case mysqlx::Value::INT64: {
                int value = wrapped;
                return ConceptValue(value);
            }
            case mysqlx::Value::BOOL: {
                bool value = wrapped;
                return ConceptValue(value);
            }
            case mysqlx::Value::FLOAT: {
                float value = wrapped;
                return ConceptValue(value);
            }
            case mysqlx::Value::STRING: {
                std::string value = wrapped;
                //cout << value << endl;
                return ConceptValue(value);
            }
            case mysqlx::Value::RAW: {
                // NOTE: This is the case for BIT columns. Here, we assume BIT(1)
                auto value = (bool) wrapped.getRawBytes().second;
                return ConceptValue(value);
            }
        }
    }

    vector<LongTermMemoryConduit::EntityAttribute>
    LongTermMemoryConduit::unwrap_attribute_rows(std::list<Row> rows) const {
        vector<EntityAttribute> result_map;
        for (auto &row: rows) {
            int obj_id = row[0];
            std::string attribute_name = row[1];
            ConceptValue col_value = unwrap_attribute_row_value(row[2]);
            result_map.emplace_back(EntityAttribute(obj_id, attribute_name, col_value));
        }
        return result_map;
    }

    vector<LongTermMemoryConduit::EntityAttribute>
    LongTermMemoryConduit::get_entity_attributes(int entity_id) const {
        std::list<Row> rows = {};
        for (const auto & table_name: table_names) {
            Table entity_attributes = db->getTable(table_name);
            RowResult result = entity_attributes.select("*").where("entity_id = :id").bind("id", entity_id).execute();
            auto result_rows = result.fetchAll();
            rows.insert(rows.end(),result_rows.begin(), result_rows.end());
        }

        return unwrap_attribute_rows(rows);
    }

    std::vector<LongTermMemoryConduit::EntityAttribute>
    LongTermMemoryConduit::get_entity_attributes(int entity_id, const std::string &attribute_name) const {
        std::list<Row> rows = {};
        for (const auto & table_name: table_names) {
            Table entity_attributes = db->getTable(table_name);
            RowResult result = entity_attributes.select("*").where("entity_id = :id and attribute_name = :attr").bind(
                    "id",
                    entity_id).bind(
                    "attr", attribute_name).execute();

            auto result_rows = result.fetchAll();
            rows.insert(rows.end(), result_rows.begin(), result_rows.end());
        }
        return unwrap_attribute_rows(rows);
    }

    vector<int> LongTermMemoryConduit::get_entities_with_attribute_of_value(const std::string &attribute_name,
                                                                           const int other_entity_id) const {
        Table entity_attributes = db->getTable("entity_attributes_id");
        RowResult result = entity_attributes.select("*").where(
                "attribute_value = :id and attribute_name = :attr").bind("id",
                                                                                   other_entity_id).bind(
                "attr", attribute_name).execute();
        std::list<Row> rows = result.fetchAll();
        vector<int> return_result;
        transform(rows.begin(), rows.end(), back_inserter(return_result), [this](Row row) {
            return boost::get<int>(unwrap_attribute_row_value(row[0]));
        });

        return return_result;
    }

    vector<int> LongTermMemoryConduit::get_entities_with_attribute_of_value(const std::string &attribute_name,
                                                                            const bool bool_val) const {
        Table entity_attributes = db->getTable("entity_attributes_bool");
        RowResult result = entity_attributes.select("*").where(
                "attribute_value = :val and attribute_name = :attr").bind("val",
                                                                          bool_val).bind(
                "attr", attribute_name).execute();
        std::list<Row> rows = result.fetchAll();
        vector<int> return_result;
        transform(rows.begin(), rows.end(), back_inserter(return_result), [this](Row row) {
            return boost::get<int>(unwrap_attribute_row_value(row[0]));
        });

        return return_result;
    }


    vector<int> LongTermMemoryConduit::get_entities_with_attribute_of_value(const std::string &attribute_name,
                                                                           const std::string &string_val) const {
        Table entity_attributes = db->getTable("entity_attributes_str");
        RowResult result = entity_attributes.select("*").where(
                "attribute_value = :val and attribute_name = :attr").bind("val",
                                                                                   string_val).bind(
                "attr", attribute_name).execute();
        std::list<Row> rows = result.fetchAll();
        vector<int> return_result;
        transform(rows.begin(), rows.end(), back_inserter(return_result), [this](Row row) {
            return boost::get<int>(unwrap_attribute_row_value(row[0]));
        });

        return return_result;
    }

    std::vector<int> LongTermMemoryConduit::get_all_entities() const {
        vector<int> all_obj_ids;
        Table entities = db->getTable("entities");
        RowResult rows = entities.select("*").execute();
        transform(rows.begin(), rows.end(), back_inserter(all_obj_ids), [](Row row) {
            return row[0];
        });
        return all_obj_ids;
    }

    void LongTermMemoryConduit::delete_all_entities() {


        Table table = db->getTable("entities");
        TableRemove remover = table.remove();
        remover.execute();

        // Add the robot itself back
        add_entity(1);
        int robot_concept_id = get_concept("robot");
        add_entity_attribute(1, "is_a", robot_concept_id);
        assert(entity_exists(1));
    }

    bool LongTermMemoryConduit::delete_attribute(int id) {
        return false;
    }

    bool LongTermMemoryConduit::attribute_exists(int id) {
        return false;
    }

    int LongTermMemoryConduit::get_concept(const std::string &name) {
        Table entity_attributes = db->getTable("entity_attributes_str");
        RowResult result = entity_attributes.select("*").where("attribute_name = 'concept' and attribute_value= :concept_name").bind("concept_name",
                                                                                                                  name).execute();
        std::list<Row> rows = result.fetchAll();
        auto unwrapped = unwrap_attribute_rows(rows);
        if (unwrapped.empty()) {
            int new_concept = add_entity();
            add_entity_attribute(new_concept, "concept", name);
            return new_concept;
        } else {
            return unwrapped[0].entity_id;
        }
    }


}