#pragma once

namespace actasp {

struct AspLiteral : public AspElement {
  virtual ~AspLiteral(){}
  virtual AspLiteral* literal_clone() const = 0;
};

/***
* We implement this hook so that boost:ptr_container knows how to copy heap objects
* @param object
* @return
*/
inline AspLiteral* new_clone(const AspLiteral& object) {
  return object.literal_clone();
};


typedef boost::ptr_vector<AspLiteral> LiteralContainer;

struct AspConditionalLiteral: public AspLiteral {
    const std::unique_ptr<AspLiteral> literal;
    LiteralContainer conditions;
    AspConditionalLiteral(AspLiteral &literal, LiteralContainer conditions): literal(literal.literal_clone()), conditions(conditions){}

    AspConditionalLiteral(const AspConditionalLiteral &other): literal(other.literal.get()->literal_clone()), conditions(conditions){}

    AspLiteral* literal_clone() const {
      return new AspConditionalLiteral(*this);
    }
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
};

}