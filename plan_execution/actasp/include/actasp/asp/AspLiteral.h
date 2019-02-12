#pragma once

#include <typeinfo>
#include <actasp/asp/AspTerm.h>

namespace actasp {

/**
 * Still a bit murky on the differences between Literals and Terms, but certainly
 * you can't put literals as arguments to AspFunctions...
 */
struct AspLiteral : virtual public AspElement {
  virtual ~AspLiteral(){}
  virtual AspLiteral* literal_clone() const = 0;

  virtual bool isLess(const AspLiteral &other) const = 0;

  virtual bool isEqual(const AspLiteral &other) const = 0;
};

/***
* We implement this hook so that boost:ptr_container knows how to copy heap objects
* @param object
* @return
*/
inline AspLiteral* new_clone(const AspLiteral& object) {
  return object.literal_clone();
};

inline bool operator<(const AspLiteral &lhs, const AspLiteral &rhs) {
  if (typeid(lhs) != typeid(rhs)) {
    // Literals don't have a meaningful order across types..
    assert(false);
  }
  return lhs.isLess(rhs);
}

inline bool operator==(const AspLiteral &lhs, const AspLiteral &rhs) {
  return typeid(lhs) == typeid(rhs) && lhs.isEqual(rhs);
}

/**
 * Polymorphic container type.
 */
typedef boost::ptr_vector<AspLiteral> LiteralContainer;

inline LiteralContainer literal_to_container(const AspLiteral &literal) {
  LiteralContainer literals;
  literals.push_back(literal.literal_clone());
  return literals;
  }
/** The mechanism for nested implication
 * a : b , c
 */
struct AspConditionalLiteral: public AspLiteral {
  const std::unique_ptr<AspLiteral> literal;
  LiteralContainer conditions;

  AspConditionalLiteral(AspLiteral &literal, LiteralContainer conditions): literal(literal.literal_clone()), conditions(conditions){}

  AspConditionalLiteral(const AspConditionalLiteral &other): literal(other.literal->literal_clone()), conditions(conditions){}

  AspLiteral* literal_clone() const {
    return new AspConditionalLiteral(*this);
  }

  virtual bool isLess(const AspLiteral &other) const {
    // No meaningful definition (that I can think of right now)
    assert(false);
  };

  virtual bool isEqual(const AspLiteral &other) const {
    auto downcast = static_cast<const AspConditionalLiteral &>(other);
    return literal.get() == downcast.literal.get() && conditions == downcast.conditions;
  };

};

enum RelationType {
  Less,
  LessEqual,
  Equal,
  NotEqual,
  Greater,
  GreaterEqual
};

struct BinRelation: public AspLiteral {
  const std::unique_ptr<AspTerm> left;
  const std::unique_ptr<AspTerm> right;
  const RelationType type;

  BinRelation(AspTerm &left, RelationType type, AspTerm &right): left(left.clone()), type(type), right(right.clone()) {}

  BinRelation(const BinRelation &other): left(other.left->clone()), type(other.type), right(other.right->clone()) {
  }

  std::string to_string() const {
    std::string rel;
    switch (type) {
      case Less:rel = "<";break;
      case LessEqual: rel = "<="; break;
      case Equal:rel = "=="; break;
      case NotEqual:rel = "!="; break;
      case Greater:rel = ">"; break;
      case GreaterEqual:rel = ">="; break;
    }
    return left->to_string() + " " + rel + " " + right->to_string();
  }

  AspLiteral* literal_clone() const {
    return new BinRelation(*this);
  }

  virtual bool isLess(const AspLiteral &other) const {
    // No meaningful definition (that I can think of right now)
    assert(false);
  };

  virtual bool isEqual(const AspLiteral &other) const {
    auto downcast = static_cast<const BinRelation &>(other);
    return type == downcast.type && left == downcast.left && right == downcast.right;
  };
};

}