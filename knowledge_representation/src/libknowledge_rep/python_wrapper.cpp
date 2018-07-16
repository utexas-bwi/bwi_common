#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <knowledge_representation/LongTermMemoryConduit.h>

using namespace boost::python;
using namespace knowledge_rep;

BOOST_PYTHON_MODULE (_libknowledge_rep_wrapper_cpp) {
    typedef LongTermMemoryConduit LTMC;
    class_<std::vector<int> >("PyIntList")
        .def(vector_indexing_suite<std::vector<int> >() );

    class_<LTMC::EntityAttribute>("EntityAttribute", init<int, std::string, LTMC::ConceptValue>())
        .def_readonly("entity_id", &LTMC::EntityAttribute::entity_id)
        .def_readonly("attribute_name", &LTMC::EntityAttribute::attribute_name)
        .def("get_int_value", &LTMC::EntityAttribute::get_int_value)
        .def("get_float_value", &LTMC::EntityAttribute::get_float_value)
        .def("get_bool_value", &LTMC::EntityAttribute::get_bool_value)
        .def("get_string_value", &LTMC::EntityAttribute::get_string_value);

    class_<std::vector<LTMC::EntityAttribute> >("PyAttributeList")
        .def(vector_indexing_suite<std::vector<LTMC::EntityAttribute> >() );

    class_<LongTermMemoryConduit>("LongTermMemoryConduit",
                                  init<std::string, uint, std::string, std::string, std::string>())
            .def("add_entity", static_cast<int (LTMC::*)()>(&LTMC::add_entity))
            .def("add_entity_attribute", static_cast<bool (LTMC::*)(int, const std::string &,
                                                                    const std::string &)>(&LTMC::add_entity_attribute))
            .def("remove_entity_attribute",
                 static_cast<bool (LTMC::*)(int, const std::string &)>(&LTMC::remove_entity_attribute))
            .def("get_entity_attributes", static_cast<std::vector<LTMC::EntityAttribute> (LTMC::*)(int,
                                                                                                  const std::string &) const>(&LTMC::get_entity_attributes) )
            .def("get_entity_attributes",
                 static_cast<std::vector<LTMC::EntityAttribute> (LTMC::*)(int) const> (&LTMC::get_entity_attributes))
            .def("get_entities_with_attribute_of_value", 
                 static_cast<std::vector<int> (LTMC::*)(const std::string &, int) const>(&LTMC::get_entities_with_attribute_of_value))
            .def("delete_entity", static_cast<bool (LTMC::*)(int)>(&LTMC::delete_entity))
            .def("entity_exists", static_cast<bool (LTMC::*)(int) const>(&LTMC::entity_exists))
            .def("delete_all_entities", static_cast<void (LTMC::*)()>(&LTMC::delete_all_entities))
                    //.def("add_entity_attribute",
                    //     static_cast<bool (LTMC::*)(int, const std::string &, const char[])>(&LTMC::add_entity_attribute))
            .def("add_entity_attribute",
                 static_cast<bool (LTMC::*)(int, const std::string &, int)>(&LTMC::add_entity_attribute))
                    //.def("add_entity_attribute",
                    //     static_cast<bool (LTMC::*)(int, const std::string &, bool)>(&LTMC::add_entity_attribute))
                    // FIXME: If we make this available to python, we can't use the int one, which is the more common
                    // anyways...
//.def("add_entity_attribute",
//                 static_cast<bool (LTMC::*)(int, const std::string &, float)>(&LTMC::add_entity_attribute))
            .def("select_query", static_cast<bool (LTMC::*)(const std::string &, std::vector<LTMC::EntityAttribute> &) const>(&LTMC::select_query))
            .def("get_all_entities", static_cast<std::vector<int> (LTMC::*)() const>(&LTMC::get_all_entities))
            .def("get_concept", static_cast<int (LTMC::*)(const std::string &)>(&LTMC::get_concept))
            .def("remove_concept_references", static_cast<bool (LTMC::*)(const std::string &)>(&LTMC::remove_concept_references))
            .def("remove_entities_of_concept", static_cast<bool (LTMC::*)(const std::string &)>(&LTMC::remove_entities_of_concept))
            .def("remove_children_of_entity", static_cast<bool (LTMC::*)(int)>(&LTMC::remove_children_of_entity));

}