#ifndef KNOWLEDGE_REPRESENTATION_ENTITYATTRIBUTE_H
#define KNOWLEDGE_REPRESENTATION_ENTITYATTRIBUTE_H

#include <string>

namespace knowledge_rep {

typedef boost::variant<int, float, bool, std::string> AttributeValue;

static std::string to_string(const AttributeValue &attribute_value) {
    switch (attribute_value.which()) {
        case 0:
            return std::to_string(boost::get<int>(attribute_value));
        case 1:
            return std::to_string(boost::get<float>(attribute_value));
        case 2:
            return std::to_string(boost::get<bool>(attribute_value));
        case 3:
            return boost::get<std::string>(attribute_value);
        default:
            assert(false);
    }
}

struct EntityAttribute {
  int entity_id;
  std::string attribute_name;
  AttributeValue value;

  EntityAttribute(int entity_id, std::string attribute_name, AttributeValue value) : entity_id(entity_id),
                                                                                   attribute_name(std::move(
                                                                                       attribute_name)),
                                                                                   value(value) {}

  int get_int_value() { return boost::get<int>(value); }

  float get_float_value() { return boost::get<float>(value); }

  bool get_bool_value() { return boost::get<bool>(value); }

  std::string get_string_value() { return boost::get<std::string>(value); }

  bool operator==(const EntityAttribute &other) {
    return (this->entity_id == other.entity_id)
           && (this->attribute_name == other.attribute_name)
           && (this->value == other.value);
  }

};
}

#endif //KNOWLEDGE_REPRESENTATION_ENTITYATTRIBUTE_H
