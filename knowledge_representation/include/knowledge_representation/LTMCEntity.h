#pragma once
#include <string>
#include <vector>
#include <knowledge_representation/LongTermMemoryConduitInterface.h>

namespace knowledge_rep {


  template <typename LTMCImpl>
  class LTMCEntity {

  public:
    int entity_id;

    LTMCEntity(int entity_id, LongTermMemoryConduitInterface<LTMCImpl> &ltmc): entity_id(entity_id), ltmc(ltmc){}

    bool add_attribute(const std::string &attribute_name, const std::string &string_val){
      return ltmc.get().add_attribute(*this, attribute_name, string_val);
    };

    bool add_attribute(const std::string &attribute_name, const char string_val[]){
      return ltmc.get().add_attribute(*this, attribute_name, std::string(string_val));
    };

    bool add_attribute(const std::string &attribute_name, float float_val){
      return ltmc.get().add_attribute(*this, attribute_name, float_val);
    };

    bool add_attribute(const std::string &attribute_name, bool bool_val){
      return ltmc.get().add_attribute(*this, attribute_name, bool_val);
    };

    bool add_attribute(const std::string &attribute_name, int other_entity_id){
      return ltmc.get().add_attribute(*this, attribute_name, other_entity_id);
    };

    bool add_attribute(const std::string &attribute_name, const LTMCEntity &other_entity){
      return ltmc.get().add_attribute(*this, attribute_name, other_entity.entity_id);
    };


    int remove_attribute(const std::string &attribute_name){
      return ltmc.get().remove_attribute(*this, attribute_name);
    };

    int remove_attribute_of_value(const std::string &attribute_name, const LTMCEntity &other_entity){
      return ltmc.get().remove_attribute_of_value(*this, attribute_name, other_entity);
    };

    std::vector<EntityAttribute> get_attributes() const {
      return ltmc.get().get_attributes(*this);
    };

    std::vector<EntityAttribute> get_attributes(const std::string &attribute_name) const{
      return ltmc.get().get_attributes(*this, attribute_name);
    };

    boost::optional<std::string> get_name() {
      // There should only be one
      auto name_attrs = get_attributes("name");
      if (!name_attrs.empty()) {
        return boost::get<std::string>(name_attrs[0].value);
      } else {
        return {};
      }
    };

    bool delete_entity(){
      return ltmc.get().delete_entity(*this);
    };

    bool is_valid() const {
      return ltmc.get().is_valid(*this);
    };

    bool operator==(const LTMCEntity &other) const {
      return this->entity_id == other.entity_id;
    }

    LTMCEntity& operator =(const LTMCEntity &that) {
      this->entity_id = that.entity_id;
      this->ltmc = that.ltmc;
    }

    std::vector<EntityAttribute> operator [](const std::string &attr_name) const {
      return get_attributes(attr_name);
    };


  protected:
    std::reference_wrapper<LongTermMemoryConduitInterface<LTMCImpl>> ltmc;


  };

}