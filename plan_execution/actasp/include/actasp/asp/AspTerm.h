#pragma once

#include <actasp/asp/AspElement.h>
#include <string>
#include <vector>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/variant.hpp>

namespace actasp {

// The "ASP-Core-2 Input Language Format" paper lays out the ASP grammar in detail
// https://www.mat.unical.it/aspcomp2013/files/ASP-CORE-2.03c.pdf

/***
 * Terms are simplest the building blocks of ASP programs. Mostly, they're used
 * as arguments to atoms (AKA AspFunctions that appear in the the head of rules or as facts).
 */
struct AspTerm : public AspElement {
  virtual std::string to_string() const = 0;

  virtual AspTerm* clone() const = 0;

  virtual bool isEqual(const AspTerm &other) const =0;

  virtual bool isLess(const AspTerm &other) const =0;

  virtual int totalOrderIndex() const = 0;
};

/***
 * We implement this hook so that boost:ptr_container knows how to copy heap objects
 * @param object
 * @return
 */
inline AspTerm* new_clone(const AspTerm& object) {
  return object.clone();
};


struct Variable;
struct SymbolicConstant;
struct SimpleTerm: public AspTerm {


  static int32_t get_int(const AspTerm &term);

  static std::string get_string(const AspTerm &term);

  static Variable get_variable(const AspTerm &term);

  static SymbolicConstant get_constant(const AspTerm &term);
};

struct StringTerm: public SimpleTerm {
  const std::string value;
    StringTerm(std::string value): value(std::move(value)) {}

    std::string to_string() const override {
      return "\"" + value + "\"";
    }

    inline AspTerm* clone() const override {
      return new StringTerm(*this);
    }

    bool isLess(const AspTerm &other) const override {
      return value < static_cast<const StringTerm&>(other).value;
    }

    bool isEqual(const AspTerm &other) const override {
      return value == static_cast<const StringTerm&>(other).value;
    }

    int totalOrderIndex() const override {
      return 2;
    }

};

struct IntTerm: public SimpleTerm {
  const int32_t value;
  IntTerm(int32_t value): value(value){};

  std::string to_string() const override {
    return std::to_string(value);
  }

  inline AspTerm* clone() const override {
    return new IntTerm(*this);
  }

  bool isLess(const AspTerm &other) const override {
    return value < static_cast<const IntTerm&>(other).value;
  }

  bool isEqual(const AspTerm &other) const override {
    return value == static_cast<const IntTerm&>(other).value;
  }

  int totalOrderIndex() const override {
    return 0;
  }

};

struct Variable: public SimpleTerm {
  std::string name;

  explicit Variable(std::string name): name(std::move(name)){}

  friend std::ostream& operator<< (std::ostream& stream, const Variable& variable) {
    stream << variable.name;
  }

  bool operator<(const Variable& other) const noexcept{
    return name < other.name;
  }

  bool operator==(const Variable& other) const noexcept {
    return name == other.name;
  }

  std::string to_string() const override {
    return name;
  }

  inline AspTerm* clone() const override {
    return new Variable(*this);
  }

  bool isLess(const AspTerm &other) const override {
    return name < static_cast<const Variable&>(other).name;
  }

  bool isEqual(const AspTerm &other) const override {
    return name == static_cast<const Variable&>(other).name;
  }

  int totalOrderIndex() const override {
    return 1;
  }

};

struct SymbolicConstant: public SimpleTerm {
  std::string name;

  explicit SymbolicConstant(std::string name): name(std::move(name)){}

  friend std::ostream& operator<< (std::ostream& stream, const SymbolicConstant& symbolic_constant) {
    stream << symbolic_constant.name;
  }

  bool operator<(const Variable& other) const noexcept{
    return name < other.name;
  }

  bool operator==(const Variable& other) const noexcept {
    return name == other.name;
  }

  std::string to_string() const override {
    return name;
  }

  inline AspTerm* clone() const override {
    return new SymbolicConstant(*this);
  }

  bool isLess(const AspTerm &other) const override {
    return name < static_cast<const SymbolicConstant&>(other).name;
  }

  bool isEqual(const AspTerm &other) const override {
    return name == static_cast<const SymbolicConstant&>(other).name;
  }

  int totalOrderIndex() const override {
    return 1;
  }

};

inline bool operator<(const AspTerm &lhs, const AspTerm &rhs) {
  return lhs.totalOrderIndex() < rhs.totalOrderIndex() || (lhs.totalOrderIndex() == rhs.totalOrderIndex() && lhs.isLess(rhs));
}

inline bool operator==(const AspTerm &lhs, const AspTerm &rhs) {
  return typeid(lhs) == typeid(rhs) && lhs.isEqual(rhs);
}




typedef boost::ptr_vector<AspTerm> TermContainer;

class AspFunction;
// Until this becomes important, it's simpler to just allow functions
// A function without arguments is infact equivalent to a constant anyways
//typedef boost::variant<AspFunction, AspConstant> AspAtom;
typedef AspFunction AspAtom;

}