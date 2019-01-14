#include <utility>

#include <utility>

#pragma once

#include <sstream>
#include <actasp/AspRule.h>
#include <actasp/AspAtom.h>

namespace actasp {

class AspProgram {

public:
  AspProgram(std::string name, std::vector<AspFluentRule> rules, std::vector<Variable> variables = {}): name(
      std::move(name)), rules(std::move(rules)), variables(std::move(variables)) {}

  const std::vector<AspFluentRule> & getRules() const{
    return rules;
  }

  const std::vector<Variable>& getVariables() const {
    return variables;
  }

  const std::string& getName() const {
    return name;
  }

  std::string to_string() const{
    std::stringstream stream;
    stream << "#program " << name;
    if (!variables.empty()) {
      stream << "(";
      bool past_first = false;
      for (const auto &variable: variables) {
        if (past_first) {
          stream << ", ";
        }
        stream << variable;
        past_first = true;
      }
      stream << ")";
    }
    stream << "." << std::endl;
    for (const auto &rule: rules) {
      stream << rule.to_string();
    }
    return stream.str();
  }


private:
  std::vector<AspFluentRule> rules;
  std::string name;
  std::vector<Variable> variables;
};


}