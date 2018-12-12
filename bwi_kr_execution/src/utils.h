#ifndef BWI_KR_EXECUTION_UTILS_H
#define BWI_KR_EXECUTION_UTILS_H

#include <knowledge_representation/MemoryConduit.h>
#include <knowledge_representation/convenience.h>
#include <iostream>
#include <actasp/AspFluent.h>


namespace bwi_krexec {
std::string memoryConduitToAsp() {
  std::vector<int> relevant_objs;
  knowledge_rep::MemoryConduit memory;
  std::vector <knowledge_rep::EntityAttribute> entity_attributes = memory.relevant_to(
      relevant_objs);

  std::set<int> all_objs;
  for (const auto &obj_attr: entity_attributes) {
    all_objs.insert(obj_attr.entity_id);
  }

  std::ostringstream out;

  out << "#program base." << std::endl;

  for (const auto obj_id: all_objs) {
    out << "entity(" << obj_id << ")." << std::endl;
  }
  for (const auto &entity_attribute: entity_attributes) {
    if (entity_attribute.value.type() == typeid(bool)) {
      if (!boost::get<bool>(entity_attribute.value)) {
        out << "-";
      }
      out << entity_attribute.attribute_name << "(" << entity_attribute.entity_id << ")." << std::endl;
    } else if (entity_attribute.value.type() == typeid(int)) {
      out << entity_attribute.attribute_name << "(" << entity_attribute.entity_id << ", "
          << entity_attribute.value << ")." << std::endl;
    } else if (entity_attribute.value.type() == typeid(std::string)) {
      out << entity_attribute.attribute_name << "(" << entity_attribute.entity_id << ", \""
          << entity_attribute.value << "\")." << std::endl;
    } else if (entity_attribute.value.type() == typeid(float)) {
      std::cout << "Skipping float entry: " << entity_attribute.entity_id << " "
                << entity_attribute.attribute_name << " " << entity_attribute.value << std::endl;
    } else {
      std::cout << "Skipping entry: " << entity_attribute.entity_id << " " << entity_attribute.attribute_name
                << " " << entity_attribute.value << std::endl;
    }


  }
  return out.str();
}

struct PrintFluent {

  PrintFluent(std::ostream &stream) : stream(stream) {}

  std::string operator()(const actasp::AspFluent &fluent) {
    stream << fluent.toString() << " ";
  }

  std::ostream &stream;

};
}

#endif //BWI_KR_EXECUTION_UTILS_H
