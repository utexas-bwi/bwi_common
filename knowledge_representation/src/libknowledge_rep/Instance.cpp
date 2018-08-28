#include <knowledge_representation/Instance.h>
#include <knowledge_representation/Concept.h>


namespace knowledge_rep {

boost::optional<std::string> Instance::get_name() {
  if (!name.empty()) {
    return name;
  }
  // There should only be one
  auto name_attrs = get_attributes("name");
  if (!name_attrs.empty()) {
    name = boost::get<std::string>(name_attrs[0].value);
    return name;
  }
  return {};
}

bool Instance::make_instance_of(const Concept &concept) {
  return add_attribute("instance_of", concept.entity_id);
}

/**
 * @brief Get all concepts that this instance is transitively an instance of
 * For example, if an entity A "instance_of" concept named apple, and apple "is_a" concept of fruit,
 * then get_concepts will return the concepts of both apple and fruit.
 * @return
 */
std::vector<Concept> Instance::get_concepts() {
  auto results = ltmc.get().sess->sql("CALL get_concepts(?)").bind(entity_id).execute();
  auto rows = results.fetchAll();
  std::vector<Concept> concepts{};
  std::transform(rows.begin(), rows.end(), std::back_inserter(concepts), [this](const mysqlx::Row &row) {
      return Concept(row[0], this->ltmc.get());
  });
  return concepts;
}
}
