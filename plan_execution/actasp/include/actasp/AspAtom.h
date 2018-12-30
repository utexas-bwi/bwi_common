#include <utility>

#include <utility>

#include <utility>

#pragma once

#include <string>
#include <stdexcept>
#include <vector>
#include <boost/variant.hpp>

namespace actasp {

struct Variable {
  std::string name;
  Variable(std::string name): name(std::move(name)){}

  friend std::ostream& operator<< (std::ostream& stream, const Variable& variable) {
    stream << variable.name;
  }

  bool operator<(const Variable& other) const noexcept{
    return name < other.name;
  }

  bool operator==(const Variable& other) const noexcept {
    return name == other.name;
  }


};


std::ostream& operator<< (std::ostream& stream, const Variable& variable);

  
class AspAtom {
public:

  typedef boost::variant<int32_t, std::string, Variable> Argument;

  AspAtom(std::string name, std::vector<Argument> variables): name(std::move(name)), variables(std::move(variables)) {}

  AspAtom(const std::string& formula);

  unsigned int arity() const noexcept;

  std::string getName() const noexcept;
  
  virtual std::vector<Argument> getParameters() const noexcept;

  virtual std::string to_string() const noexcept;
  
  virtual operator std::string() const { return this->to_string(); } 

  virtual ~AspAtom() = default;

  bool operator<(const AspAtom& other) const noexcept{
    return name < other.name ||  ((name == other.name) && variables < other.variables);
  }
  bool operator>(const AspAtom& other) const noexcept{
    return name > other.name ||  ((name == other.name) && variables > other.variables);
  }

  bool operator==(const AspAtom& other) const noexcept {
    return name == other.name && variables == other.variables;
  }

protected:
  std::string name;
  std::vector<Argument> variables;
};

AspAtom operator ""_a(const char *string, std::size_t size);

std::ostream& operator<< (std::ostream& stream, const AspAtom& atom);

}

