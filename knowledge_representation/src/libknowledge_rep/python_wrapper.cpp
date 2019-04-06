#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/noncopyable.hpp>
#include <boost/mpl/if.hpp>
#include <boost/optional.hpp>
#include <boost/python.hpp>
#include <boost/type_traits/integral_constant.hpp>
#include <boost/utility/in_place_factory.hpp>
#include <knowledge_representation/LongTermMemoryConduitInterface.h>
#include <knowledge_representation/LTMCEntity.h>
#include <knowledge_representation/LTMCConcept.h>
#include <knowledge_representation/LTMCInstance.h>
#include <knowledge_representation/convenience.h>
namespace python = boost::python;
using python::class_;
using python::bases;
using python::init;
using python::vector_indexing_suite;
using namespace knowledge_rep;
using std::vector;
using std::string;

namespace detail {

/// @brief Type trait that determines if the provided type is
///        a boost::optional.
template<typename T>
struct is_optional : boost::false_type {
};

template<typename T>
struct is_optional<boost::optional<T> > : boost::true_type {
};

/// @brief Type used to provide meaningful compiler errors.
template<typename>
struct return_optional_requires_a_optional_return_type {
};

/// @brief ResultConverter model that converts a boost::optional object to
///        Python None if the object is empty (i.e. boost::none) or defers
///        to Boost.Python to convert object to a Python object.
template<typename T>
struct to_python_optional {
  /// @brief Only supports converting Boost.Optional types.
  /// @note This is checked at runtime.
  bool convertible() const { return detail::is_optional<T>::value; }

  /// @brief Convert boost::optional object to Python None or a
  ///        Boost.Python object.
  PyObject *operator()(const T &obj) const {
    namespace python = boost::python;
    python::object result =
        obj                      // If boost::optional has a value, then
        ? python::object(*obj) // defer to Boost.Python converter.
        : python::object();    // Otherwise, return Python None.

    // The python::object contains a handle which functions as
    // smart-pointer to the underlying PyObject.  As it will go
    // out of scope, explicitly increment the PyObject's reference
    // count, as the caller expects a non-borrowed (i.e. owned) reference.
    return python::incref(result.ptr());
  }

  /// @brief Used for documentation.
  const PyTypeObject *get_pytype() const { return 0; }
};

} // namespace detail

/// @brief Converts a boost::optional to Python None if the object is
///        equal to boost::none.  Otherwise, defers to the registered
///        type converter to returs a Boost.Python object.
struct return_optional {
  template<class T>
  struct apply {
    // The to_python_optional ResultConverter only checks if T is convertible
    // at runtime.  However, the following MPL branch cause a compile time
    // error if T is not a boost::optional by providing a type that is not a
    // ResultConverter model.
    typedef typename boost::mpl::if_<
        detail::is_optional<T>,
        detail::to_python_optional<T>,
        detail::return_optional_requires_a_optional_return_type<T>
    >::type type;
  }; // apply
};   // return_optional


BOOST_PYTHON_MODULE (_libknowledge_rep_wrapper_cpp) {
  typedef LongTermMemoryConduit LTMC;

  class_<std::pair<string, string> >("StringPair")
      .def_readwrite("first", &std::pair<string, string>::first)
      .def_readwrite("second", &std::pair<string, string>::second);

  class_<vector<std::pair<string, string> > >("PyPairList")
      .def(vector_indexing_suite<vector<std::pair<string, string>>>());

  class_<vector<Entity> >("PyEntityList")
      .def(vector_indexing_suite<vector<Entity> >());

  class_<vector<Concept> >("PyConceptList")
      .def(vector_indexing_suite<vector<Concept> >());

  class_<vector<Instance> >("PyInstanceList")
      .def(vector_indexing_suite<vector<Instance> >());

  class_<Entity>("Entity", init<int, LTMC &>())
      .def_readonly("entity_id", &Entity::entity_id)
      .def("add_attribute_str",
           static_cast<bool (Entity::*)(const string &, const string &)>
           (&Entity::add_attribute))
      .def("add_attribute_int",
           static_cast<bool (Entity::*)(const string &, int)>
           (&Entity::add_attribute))
      .def("add_attribute_float",
           static_cast<bool (Entity::*)(const string &, float)>
           (&Entity::add_attribute))
      .def("add_attribute_bool",
           static_cast<bool (Entity::*)(const string &, bool)>
           (&Entity::add_attribute))
      .def("add_attribute_entity",
           static_cast<bool (Entity::*)(const string &, const Entity &)>
           (&Entity::add_attribute))
      .def("remove_attribute", &Entity::remove_attribute)
      .def("get_attributes", static_cast<vector<EntityAttribute> (Entity::*)(const string &) const>
      (&Entity::get_attributes))
      .def("get_attributes",
           static_cast<vector<EntityAttribute> (Entity::*)() const> (&Entity::get_attributes))
      .def("delete", &Entity::delete_entity)
      .def("is_valid", &Entity::is_valid);

  class_<Concept, bases<Entity> >("Concept", init<int, LTMC &>())
      .def("remove_instances", &Concept::remove_instances)
      .def("remove_references", &Concept::remove_references)
      .def("get_instances", &Concept::get_instances)
      .def("get_name", &Concept::get_name)
      .def("get_children", &Concept::get_children)
      .def("create_instance", static_cast<Instance (Concept::*)()> (&Concept::create_instance))
      .def("create_instance",
           static_cast<boost::optional<Instance> (Concept::*)(const string &)> (&Concept::create_instance),
           python::return_value_policy<return_optional>());

  class_<Instance, bases<Entity> >("Instance", init<int, LTMC &>())
      .def("make_instance_of", &Instance::make_instance_of)
      .def("get_name", &Instance::get_name,
           python::return_value_policy<return_optional>())
      .def("get_concepts", &Instance::get_concepts)
      .def("has_concept", &Instance::has_concept);


  class_<EntityAttribute>("EntityAttribute", init<int, string, AttributeValue>())
      .def_readonly("entity_id", &EntityAttribute::entity_id)
      .def_readonly("attribute_name", &EntityAttribute::attribute_name)
      .def("get_int_value", &EntityAttribute::get_int_value)
      .def("get_float_value", &EntityAttribute::get_float_value)
      .def("get_bool_value", &EntityAttribute::get_bool_value)
      .def("get_string_value", &EntityAttribute::get_string_value);

  class_<vector<EntityAttribute> >("PyAttributeList")
      .def(vector_indexing_suite<vector<EntityAttribute> >());

  class_<LongTermMemoryConduit, boost::noncopyable>("LongTermMemoryConduit",
                                                    init<const string &>())
      .def("add_entity", static_cast<Entity (LTMC::*)()>(&LTMC::add_entity))
      .def("entity_exists", &LTMC::entity_exists)
      .def("delete_all_entities", &LTMC::delete_all_entities)
      .def("get_entities_with_attribute_of_value",
           static_cast<vector<Entity> (LTMC::*)(const string &, const int)>
           (&LTMC::get_entities_with_attribute_of_value))
      .def("select_query_int", static_cast<bool (LTMC::*)(const string &,
                                                          vector<EntityAttribute> &) const>(&LTMC::select_query_int))
      .def("select_query_bool", static_cast<bool (LTMC::*)(const string &,
                                                           vector<EntityAttribute> &) const>(&LTMC::select_query_bool))
      .def("select_query_float", static_cast<bool (LTMC::*)(const string &,
                                                            vector<EntityAttribute> &) const>(&LTMC::select_query_float))
      .def("select_query_string", static_cast<bool (LTMC::*)(const string &,
                                                             vector<EntityAttribute> &) const>(&LTMC::select_query_string))
      .def("get_all_entities", &LTMC::get_all_entities)
      .def("get_all_concepts", &LTMC::get_all_concepts)
      .def("get_all_instances", &LTMC::get_all_instances)
      .def("get_all_attribute_names", &LTMC::get_all_attributes)
      .def("get_concept", &LTMC::get_concept)
      .def("get_instance_named", &LTMC::get_instance_named)
      .def("get_robot", &LTMC::get_robot);

}