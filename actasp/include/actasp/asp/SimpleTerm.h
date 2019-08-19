#pragma once

#include <actasp/asp/AspTerm.h>

namespace actasp {
struct Variable;
struct SymbolicConstant;

/***
 * SimpleTerms are thin wrappers around fundamental types
 */
struct SimpleTerm : public AspTerm {

  static int32_t get_int(const AspTerm &term);

  static std::string get_string(const AspTerm &term);

  static Variable get_variable(const AspTerm &term);

  static SymbolicConstant get_constant(const AspTerm &term);
};

struct StringTerm : public SimpleTerm {
  const std::string value;

  StringTerm(std::string value) : value(std::move(value)) {}

  std::string to_string() const override {
    return "\"" + value + "\"";
  }

  inline AspTerm *clone() const override {
    return new StringTerm(*this);
  }

  bool isLess(const AspTerm &other) const override {
    return value < static_cast<const StringTerm &>(other).value;
  }

  bool isEqual(const AspTerm &other) const override {
    return value == static_cast<const StringTerm &>(other).value;
  }

  int totalOrderIndex() const override {
    return 2;
  }

};

struct IntTerm : public SimpleTerm {
  const int32_t value;

  IntTerm(int32_t value) : value(value) {};

  std::string to_string() const override {
    return std::to_string(value);
  }

  inline AspTerm *clone() const override {
    return new IntTerm(*this);
  }

  bool isLess(const AspTerm &other) const override {
    return value < static_cast<const IntTerm &>(other).value;
  }

  bool isEqual(const AspTerm &other) const override {
    return value == static_cast<const IntTerm &>(other).value;
  }

  int totalOrderIndex() const override {
    return 0;
  }

};

struct Variable : public SimpleTerm {
  std::string name;

  explicit Variable(std::string name) : name(std::move(name)) {}

  friend std::ostream &operator<<(std::ostream &stream, const Variable &variable) {
    stream << variable.name;
  }

  bool operator<(const Variable &other) const noexcept {
    return name < other.name;
  }

  bool operator==(const Variable &other) const noexcept {
    return name == other.name;
  }

  std::string to_string() const override {
    return name;
  }

  inline AspTerm *clone() const override {
    return new Variable(*this);
  }

  bool isLess(const AspTerm &other) const override {
    return name < static_cast<const Variable &>(other).name;
  }

  bool isEqual(const AspTerm &other) const override {
    return name == static_cast<const Variable &>(other).name;
  }

  int totalOrderIndex() const override {
    return 1;
  }

};

struct SymbolicConstant : public SimpleTerm {
  std::string name;

  explicit SymbolicConstant(std::string name) : name(std::move(name)) {}

  friend std::ostream &operator<<(std::ostream &stream, const SymbolicConstant &symbolic_constant) {
    stream << symbolic_constant.name;
  }

  bool operator<(const Variable &other) const noexcept {
    return name < other.name;
  }

  bool operator==(const Variable &other) const noexcept {
    return name == other.name;
  }

  std::string to_string() const override {
    return name;
  }

  inline AspTerm *clone() const override {
    return new SymbolicConstant(*this);
  }

  bool isLess(const AspTerm &other) const override {
    return name < static_cast<const SymbolicConstant &>(other).name;
  }

  bool isEqual(const AspTerm &other) const override {
    return name == static_cast<const SymbolicConstant &>(other).name;
  }

  int totalOrderIndex() const override {
    return 1;
  }

};

/**
 * @brief Represents a range of values.
 * e.g:   bit(1..n)
 */
struct Interval : public AspTerm {
  const std::unique_ptr <AspTerm> left;
  const std::unique_ptr <AspTerm> right;

  Interval(const AspTerm &left, const AspTerm &right) : left(left.clone()), right(right.clone()) {}

  AspTerm *clone() const override {
    return new Interval(*left, *right);
  }

  bool isEqual(const AspTerm &other) const override {
    const auto &downcast = static_cast<const Interval &>(other);
    return left->isEqual(*downcast.left) && right->isEqual(*downcast.right);
  }

  bool isLess(const AspTerm &other) const override {
    const auto &downcast = static_cast<const Interval &>(other);
    return left->isLess(*downcast.left) || right->isLess(*downcast.right);
  }

  std::string to_string() const override {
    return left->to_string() + ".." + right->to_string();
  }

  int totalOrderIndex() const override {
    assert(false);
  }
};

inline bool operator<(const AspTerm &lhs, const AspTerm &rhs) {
  return lhs.totalOrderIndex() < rhs.totalOrderIndex() ||
         (lhs.totalOrderIndex() == rhs.totalOrderIndex() && lhs.isLess(rhs));
}

inline bool operator==(const AspTerm &lhs, const AspTerm &rhs) {
  return typeid(lhs) == typeid(rhs) && lhs.isEqual(rhs);
}
}