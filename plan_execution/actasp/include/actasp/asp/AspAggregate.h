#pragma once

#include <actasp/asp/AspElement.h>
#include <actasp/asp/AspTerm.h>
#include <actasp/asp/AspLiteral.h>

namespace actasp {

struct AggregateRelation: public AspElement {
  const RelationType type;
  const std::unique_ptr<AspTerm> term;
  AggregateRelation(RelationType type, const AspTerm &term): type(type), term(term.clone()){};

  AggregateRelation(const AggregateRelation &other): type(other.type), term(other.term.get()->clone()) {}
};

struct AggregateElement: public AspElement {
  const TermContainer terms;
  const LiteralContainer literals;
  AggregateElement(TermContainer terms, LiteralContainer literals){};
};
typedef std::vector<AggregateElement> AggregateElementList;

enum AggregateType {
  Count,
  Min,
  Max,
  Sum,
  SumPositive
};

struct AspAggregate: public AspElement {
  const AggregateRelation lower_relation;
  const AggregateRelation upper_relation;
  const AggregateType type;
  const std::vector<AggregateElement> elements;

  AspAggregate(const IntTerm &lower_term, AggregateElementList elements, const IntTerm &upper_term): type(Count),
                                                                                         lower_relation(LessEqual, lower_term), elements(elements), upper_relation(LessEqual, upper_term){}
  AspAggregate(AspTerm &lower_term, AggregateElementList elements, AspTerm &upper_term): type(Count),
  lower_relation(LessEqual, lower_term), elements(elements), upper_relation(LessEqual, upper_term){}



};

//TODO: Aggregate rules. Aggregates in the body
}