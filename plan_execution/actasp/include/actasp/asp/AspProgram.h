#pragma once

#include <sstream>
#include <actasp/asp/AspRule.h>
#include <actasp/asp/AspAtom.h>
#include <actasp/asp/AspElement.h>

namespace actasp {

/**
 * @brief A fragment of an ASP program. Represented as an unowned collection of elements.
 */
class AspProgram {

public:
  AspProgram(std::string name, std::vector<AspElement *> rules, std::vector<Variable> variables = {}): name(
      std::move(name)), rules(std::move(rules)), variables(std::move(variables)) {}

  const std::vector<AspElement*> & get_elements() const{
    return rules;
  }

  const std::vector<Variable>& getVariables() const {
    return variables;
  }

  const std::string& getName() const {
    return name;
  }

private:
  std::vector<AspElement*> rules; //unowned
  std::string name;
  std::vector<Variable> variables;
};


}