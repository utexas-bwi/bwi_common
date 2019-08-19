#include <utility>

#pragma once

#include <sstream>
#include <actasp/asp/AspRule.h>
#include <actasp/asp/AspFunction.h>
#include <actasp/asp/AspElement.h>

namespace actasp {

/**
 * @brief A fragment of an ASP program. Represented as an unowned collection of elements.
 */
struct AspProgram {

  AspProgram(std::string name, std::vector<AspElement *> rules, std::vector<Variable> variables = {}): name(
      std::move(name)), user_facing_name(this->name), elements(std::move(rules)), variables(std::move(variables)) {}

  AspProgram(std::string name, std::string user_facing_name, std::vector<AspElement *> rules, std::vector<Variable> variables = {}): name(
      std::move(name)), user_facing_name(std::move(user_facing_name)), elements(std::move(rules)), variables(std::move(variables)) {}

      // FIXME: This pattern is pretty dangerous, and it'll probably lead to a bug if
      // the type is used carelessly outside of the Clingo instance. Maybe should provide
      // a safe implementation too?
  const std::vector<AspElement*> elements; //unowned
  const std::string name;
  // If the program has a more meaningful name than the clingo name (which must be one of check, step, or base
  // for regular planning programs), user can put it here
  const std::string user_facing_name;
  const std::vector<Variable> variables;
};


}