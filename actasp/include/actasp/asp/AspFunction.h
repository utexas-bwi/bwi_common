#pragma once

#include <string>
#include <stdexcept>
#include <vector>
#include <boost/variant.hpp>
#include <boost/iterator/indirect_iterator.hpp>
#include <actasp/asp/AspTerm.h>
#include <actasp/asp/SimpleTerm.h>
#include <actasp/asp/AspLiteral.h>
#include <clingo.hh>

namespace actasp {


std::ostream &operator<<(std::ostream &stream, const Variable &variable);


/***
 * Functions are a named tuple of ASP terms.
 * Ex:  at(home, time(12), X)
 * Ex:  sum(X, Y)
 * Ex:  -open("door_312", 0)
 */
struct AspFunction : public AspTerm {

  std::string name;
  TermContainer arguments;
  bool negative;


  AspFunction(std::string name, TermContainer arguments, bool negative = false) : name(std::move(name)),
                                                                                  arguments(std::move(arguments)),
                                                                                  negative(negative) {}

  static AspFunction from_symbol(const Clingo::Symbol &symbol);

  static AspFunction from_string(const char *string);

  unsigned int arity() const noexcept;

  virtual std::string to_string() const noexcept;

  virtual explicit operator std::string() const { return this->to_string(); }

  inline AspTerm *clone() const override {
    return new AspFunction(*this);
  }

  ~AspFunction() override = default;

  bool isLess(const AspTerm &other) const override {
    return *this < static_cast<const AspFunction &>(other);
  }

  bool isEqual(const AspTerm &other) const override {
    return *this == static_cast<const AspFunction &>(other);
  }

  int totalOrderIndex() const override {
    return 4;
  }

  bool operator<(const AspFunction &other) const noexcept {
    return negative < other.negative || name < other.name || ((name == other.name) && arguments < other.arguments);
  }

  bool operator>(const AspFunction &other) const noexcept {
    return negative > other.negative || name > other.name || ((name == other.name) && arguments > other.arguments);
  }

  bool operator==(const AspFunction &other) const noexcept {
    const auto test = std::equal_to<AspTerm>();
    return negative == other.negative &&
           name == other.name && arguments.size() == other.arguments.size() &&
           std::equal(arguments.begin(), arguments.end(), other.arguments.begin(),
                      [](const AspTerm &lhs, const AspTerm &rhs) { return lhs == rhs; });
  }

};

AspFunction operator ""_a(const char *string, std::size_t size);

std::ostream &operator<<(std::ostream &stream, const AspFunction &atom);

// Until this becomes important, it's simpler to just allow functions.
// A function without arguments is infact equivalent to a constant anyways
//typedef boost::variant<AspFunction, AspConstant> AspAtom;

typedef AspFunction AspAtom;

// TODO: Reconsider how to handle wrapping so we can easily support aggregate literals...
/**
 * @brief A helper class for wrapping up atoms or fluents as literals
 * @tparam T
 */
template<class T>
struct WrappedAtom : public AspLiteral {
  const T atom;

  explicit WrappedAtom(T atom, NegationType negation=NegationType::None) : atom(std::move(atom)), AspLiteral(negation) {
  }

  bool isLess(const AspLiteral &other) const {
    return negation < other.negation || this->atom < static_cast<const WrappedAtom<T> &>(other).atom;
  };

  bool isEqual(const AspLiteral &other) const {
    return negation == other.negation && this->atom == static_cast<const WrappedAtom<T> &>(other).atom;
  };

  AspLiteral *literal_clone() const override {
    return new WrappedAtom<T>(*this);
  }

  std::string to_string() const {
    std::stringstream sstream;
    sstream << negation << atom.to_string();
    return sstream.str();
  }

  static WrappedAtom<T> from_string(const char *string) {
    std::string as_str(string);
    size_t pos = 0;
    while (std::string::npos != (pos = as_str.find('\'', pos)))
    {
      as_str.replace(pos, 1, "\"", 1);
      pos += 1;
    }
      std::unique_ptr<WrappedAtom<T>> result;
      using namespace Clingo::AST;
      auto callback = [&result](Statement statement) {
        if (statement.data.is<Rule>()) {
          Rule as_rule = statement.data.get<Rule>();
          Literal as_literal = as_rule.head.data.get<Literal>();
          Term term = as_literal.data.get<Term>();
          auto parsed_term = *static_cast<T*>(AspTerm::from_ast(term).get());
          result = std::unique_ptr<WrappedAtom<T>>(new WrappedAtom<T>(parsed_term, as_literal.sign));

        }
    };
    Clingo::parse_program(as_str.append(".").c_str(),callback);
    return *result;

  }
};

typedef WrappedAtom<AspAtom> AtomLiteral;

/**
 * @brief A custom operator for making a literal from a string
 * @param string
 * @param size
 * @return
 *
 * Example: "not -bit_on(1, n)"_al
 */
AtomLiteral operator ""_al(const char *string, std::size_t size);

inline LiteralContainer literals_to_container(const std::vector<AtomLiteral> &literals_vector) {
  LiteralContainer literals;
  for (const auto &atom_literal: literals_vector) {
    literals.push_back(atom_literal.literal_clone());
  }
  return literals;
}
}