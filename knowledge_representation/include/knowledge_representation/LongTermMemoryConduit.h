#ifndef LONG_TERM_MEMORY_CONDUIT_H
#define LONG_TERM_MEMORY_CONDUIT_H
#include <iostream>
#include <mysqlx/xdevapi.h>
#include <string>
#include <map>
#include <boost/variant.hpp>
#include <boost/optional.hpp>
#include <utility>

namespace knowledge_rep {
    class LongTermMemoryConduit {

        std::shared_ptr<mysqlx::Session> sess;
        std::shared_ptr<mysqlx::Schema> db;

    public:
        typedef boost::variant<int, float, bool, std::string> ConceptValue;

        struct EntityAttribute {
            int entity_id;
            std::string attribute_name;
            ConceptValue value;

            EntityAttribute(int entity_id, std::string attribute_name, ConceptValue value) : entity_id(entity_id),
                                                                                             attribute_name(std::move(
                                                                                                     attribute_name)),
                                                                                             value(value) {}

            int get_int_value() { return boost::get<int>(value); }
            float get_float_value() { return boost::get<float>(value); }
            bool get_bool_value() { return boost::get<bool>(value); }
            std::string get_string_value() { return boost::get<std::string>(value); }

            bool operator==(const EntityAttribute& other) {
                return (this->entity_id == other.entity_id) 
                    && (this->attribute_name == other.attribute_name) 
                    && (this->value == other.value);
            }

        };
        LongTermMemoryConduit(const std::string &addr, const uint port, const std::string &usr,
                              const std::string &password, const std::string &db_name);

        int add_entity();

        bool add_attribute(const std::string &name);

        bool add_entity_attribute(int entity_id, const std::string &attribute_name, const std::string &string_val);

        bool add_entity_attribute(int entity_id, const std::string &attribute_name, const float float_val);

        bool add_entity_attribute(int entity_id, const std::string &attribute_name, const bool bool_val);

        bool add_entity_attribute(int entity_id, const std::string &attribute_name, const int other_entity_id);

        bool remove_entity_attribute(int entity_id, const std::string &attribute_name);

        bool remove_concept_references(const std::string &concept_name);

        bool remove_children_of_entity(int entity_id);

        bool remove_entities_of_concept(const std::string &concept_name);

        std::vector<EntityAttribute> get_entity_attributes(int entity_id) const;

        std::vector<EntityAttribute> get_entity_attributes(int entity_id, const std::string &attribute_name) const;

        //TODO: Provide versions for the other possible attribute value types.
        std::vector<int>
        get_entities_with_attribute_of_value(const std::string &attribute_name, const int other_entity_id) const;

        std::vector<int>
        get_entities_with_attribute_of_value(const std::string &attribute_name, const bool other_entity_id) const;

        std::vector<int> 
        get_entities_with_attribute_of_value(const std::string &attribute_name, const std::string &string_val) const;

        bool delete_entity(int id);

        bool entity_exists(int id);

        bool delete_attribute(int id);

        bool attribute_exists(int id);

        std::string get_entity_type(int id);

        ~LongTermMemoryConduit();

        void delete_all_entities();

        bool add_entity_attribute(int entity_id, const std::string &attribute_name, const char string_val[]);

        std::vector<int> get_all_entities() const;

        //// CONVENIENCE
        int get_concept(const std::string &name);


    private:

        static const std::string table_names[];
        std::vector<EntityAttribute> unwrap_attribute_rows(std::list<mysqlx::Row> rows) const;

        LongTermMemoryConduit::ConceptValue unwrap_attribute_row_value(mysqlx::Value wrapped) const;

        LongTermMemoryConduit::ConceptValue unwrap_attribute_row(mysqlx::Row row) const;

        bool add_entity(int id);


    };
}

#endif

