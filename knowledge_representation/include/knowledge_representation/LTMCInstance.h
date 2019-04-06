#pragma once
#include <utility>

#include <knowledge_representation/LTMCEntity.h>

namespace knowledge_rep {

  template <typename LTMCImpl>
class LTMCInstance : public LTMCEntity<LTMCImpl> {

  template <typename ConLTMCImpl> class Concept;
    std::string name;
public:

    LTMCInstance(int entity_id, std::string name, LongTermMemoryConduitInterface<LTMCImpl> &ltmc) : name(std::move(name)),
                                                                                    LTMCEntity<LTMCImpl>(entity_id, ltmc) {}

    LTMCInstance(int entity_id, LongTermMemoryConduitInterface<LTMCImpl> &ltmc) : LTMCEntity<LTMCImpl> (entity_id, ltmc) {}

    boost::optional<std::string> get_name() {
      if (!name.empty()) {
        return name;
      }
      // There should only be one
      auto name_attrs = this->ltmc.get().get_attributes(*this, "name");
      if (!name_attrs.empty()) {
        name = boost::get<std::string>(name_attrs[0].value);
        return name;
      }
      return {};
    };

    bool make_instance_of(const LTMCConcept<LTMCImpl> &concept){
      return this->add_attribute("instance_of", concept.entity_id);
    }

    std::vector<LTMCConcept<LTMCImpl>> get_concepts() const{
      return this->ltmc.get().get_concepts(*this);
    }

    bool has_concept(const LTMCConcept<LTMCImpl> &concept) {
      auto concepts = this->get_concepts();
      return std::find(concepts.begin(), concepts.end(), concept) != concepts.end();
    }

};


}

