#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/noncopyable.hpp>
#include <knowledge_representation/LongTermMemoryConduit.h>
#include <knowledge_representation/Entity.h>
#include <knowledge_representation/Concept.h>

using namespace boost::python;
using namespace knowledge_rep;

BOOST_PYTHON_MODULE (_libknowledge_rep_wrapper_cpp) {
  typedef LongTermMemoryConduit LTMC;

  class_<Entity>("Entity", init<int, LTMC &>())
      .def_readonly("entity_id", &Entity::entity_id)
      .def("add_attribute_str",
           static_cast<bool (Entity::*)(const std::string &, const std::string &)>
           (&Entity::add_attribute))
      .def("add_attribute_int",
           static_cast<bool (Entity::*)(const std::string &, int)>
           (&Entity::add_attribute))
      .def("add_attribute_float",
           static_cast<bool (Entity::*)(const std::string &, float)>
           (&Entity::add_attribute))
      .def("add_attribute_bool",
           static_cast<bool (Entity::*)(const std::string &, bool)>
           (&Entity::add_attribute))
      .def("add_attribute_entity",
           static_cast<bool (Entity::*)(const std::string &, const Entity &)>
           (&Entity::add_attribute))
      .def("remove_attribute", &Entity::remove_attribute)
      .def("get_attributes", static_cast<std::vector<EntityAttribute> (Entity::*)(const std::string &) const>
      (&Entity::get_attributes))
      .def("get_attributes",
           static_cast<std::vector<EntityAttribute> (Entity::*) () const> (&Entity::get_attributes))
      .def("delete_entity", &Entity::delete_entity)
      .def("make_instance_of", &Entity::make_instance_of)
      .def("is_valid", &Entity::is_valid);

  class_<std::vector<Entity> >("PyEntityList")
      .def(vector_indexing_suite<std::vector<Entity> >());

  class_<Concept, bases<Entity> >("Concept", init<int, LTMC &>())
      .def("remove_instances", &Concept::remove_instances)
      .def("get_instances", &Concept::get_instances)
      .def("get_name", &Concept::get_name)
      .def("create_instance", static_cast<Entity (Concept::*) ()> (&Concept::create_instance));


  class_<EntityAttribute>("EntityAttribute", init<int, std::string, AttributeValue>())
      .def_readonly("entity_id", &EntityAttribute::entity_id)
      .def_readonly("attribute_name", &EntityAttribute::attribute_name)
      .def("get_int_value", &EntityAttribute::get_int_value)
      .def("get_float_value", &EntityAttribute::get_float_value)
      .def("get_bool_value", &EntityAttribute::get_bool_value)
      .def("get_string_value", &EntityAttribute::get_string_value);

  class_<std::vector<EntityAttribute> >("PyAttributeList")
      .def(vector_indexing_suite<std::vector<EntityAttribute> >());

  class_<LongTermMemoryConduit, boost::noncopyable>("LongTermMemoryConduit",
                                init<const std::string &, uint, const std::string &, const std::string &, const std::string &>())
      .def("add_entity", static_cast<Entity (LTMC::*)()>(&LTMC::add_entity))

      .def("entity_exists", static_cast<bool (LTMC::*)(int) const>(&LTMC::entity_exists))
      .def("entity_exists", static_cast<bool (LTMC::*)(const Entity&) const>(&LTMC::entity_exists))
      .def("delete_all_entities", &LTMC::delete_all_entities)
          //.def("add_entity_attribute",
          //     static_cast<bool (LTMC::*)(int, const std::string &, const char[])>(&LTMC::add_entity_attribute))
      .def("get_entities_with_attribute_of_value",
           static_cast<std::vector<Entity> (LTMC::*)(const std::string &, const int)>
           (&LTMC::get_entities_with_attribute_of_value))
      .def("select_query_int", static_cast<bool (LTMC::*) (const std::string &, std::vector<EntityAttribute> &) const>(&LTMC::select_query_int))
      .def("select_query_bool", static_cast<bool (LTMC::*) (const std::string &, std::vector<EntityAttribute> &) const>(&LTMC::select_query_bool))
      .def("select_query_float", static_cast<bool (LTMC::*) (const std::string &, std::vector<EntityAttribute> &) const>(&LTMC::select_query_float))
      .def("select_query_string", static_cast<bool (LTMC::*) (const std::string &, std::vector<EntityAttribute> &) const>(&LTMC::select_query_string))
      .def("get_all_entities", &LTMC::get_all_entities)
      .def("get_concept", &LTMC::get_concept)
      .def("get_object_named", &LTMC::get_object_named)
      .def("remove_concept_references", &LTMC::remove_concept_references);

}
