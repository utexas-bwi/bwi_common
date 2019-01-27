#include <utility>

#include <utility>

#include <boost/optional.hpp>

#pragma once

#include <string>
#include <stdexcept>
#include <vector>
#include <boost/variant.hpp>
#include <boost/iterator/indirect_iterator.hpp>
#include <actasp/asp/AspTerm.h>
#include <actasp/asp/AspLiteral.h>

namespace actasp {

enum Negation {
  Default, // "not"
  Classical // "-"
};

std::ostream& operator<< (std::ostream& stream, const Variable& variable);


/***
 * Functions are a named tuple of ASP terms.
 * Ex:  at(home, time(12), X)
 * Ex:  sum(X, Y)
 */
class AspFunction : public AspTerm, public AspLiteral {
public:


  AspFunction(std::string name, TermContainer arguments, std::vector<Negation> negation={}): name(std::move(name)), arguments(std::move(arguments)), negation(
      std::move(negation)) {}

  explicit AspFunction(const std::string& formula);

  unsigned int arity() const noexcept;

  std::string getName() const noexcept;
  
  virtual TermContainer getArguments() const noexcept;

  virtual std::vector<Negation> getNegation() const noexcept;

  virtual std::string to_string() const noexcept;
  
  virtual explicit operator std::string() const { return this->to_string(); }

  inline AspTerm* clone() const override {
    return new AspFunction(*this);
  }

  AspLiteral* literal_clone() const override {
    return new AspFunction(*this);
  }

  ~AspFunction() override = default;

  bool isLess(const AspTerm &other) const {
    return *this < static_cast<const AspFunction&>(other);
  }

  bool isEqual(const AspTerm &other) const {
    return *this == static_cast<const AspFunction&>(other);
  }

  int totalOrderIndex() const {
    return 4;
  }

  bool operator<(const AspFunction& other) const noexcept{
    return negation < other.negation || name < other.name ||  ((name == other.name) && arguments < other.arguments);
  }

  bool operator>(const AspFunction& other) const noexcept{
    return negation > other.negation || name > other.name ||  ((name == other.name) && arguments > other.arguments);
  }

  bool operator==(const AspFunction& other) const noexcept {
    const auto test = std::equal_to<AspTerm>();
    return negation == other.negation &&
    name == other.name && arguments.size() == other.arguments.size() &&
    std::equal(arguments.begin(), arguments.end(), other.arguments.begin(),
        [](const AspTerm &lhs, const AspTerm &rhs){ return lhs == rhs;});
  }

protected:
  std::string name;
  TermContainer arguments;
  std::vector<Negation> negation;
};

AspFunction operator ""_a(const char *string, std::size_t size);

std::ostream& operator<< (std::ostream& stream, const AspFunction& atom);

}



