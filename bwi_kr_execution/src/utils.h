#ifndef BWI_KR_EXECUTION_UTILS_H
#define BWI_KR_EXECUTION_UTILS_H

#include <knowledge_representation/convenience.h>
#include <knowledge_representation/LTMCConcept.h>
#include <iostream>
#include <actasp/AspFluent.h>


namespace bwi_krexec {
std::string knowledgeBaseToAsp() {
  knowledge_rep::LongTermMemoryConduit ltmc = knowledge_rep::getDefaultLTMC();
  std::vector <knowledge_rep::Instance> instances = ltmc.getAllInstances();
  std::vector <knowledge_rep::EntityAttribute> entity_attributes = ltmc.getAllEntityAttributes();

  std::ostringstream out;

  out << "#program base." << std::endl;

  for (const auto &instance: instances) {
    std::vector <knowledge_rep::Concept> concepts = instance.getConceptsRecursive();
    for (const auto &concept: concepts) {
      out << "has_concept(" << instance.entity_id << ", \"" << concept.getName() << "\")." << std::endl;
    }
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
