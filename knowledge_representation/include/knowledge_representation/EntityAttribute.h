#ifndef KNOWLEDGE_REPRESENTATION_ENTITYATTRIBUTE_H
#define KNOWLEDGE_REPRESENTATION_ENTITYATTRIBUTE_H


namespace knowledge_rep {

typedef boost::variant<int, float, bool, std::string> AttributeValue;

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
