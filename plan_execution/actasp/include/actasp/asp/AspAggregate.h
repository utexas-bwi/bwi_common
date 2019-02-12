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
  const LiteralContainer conditions;

  AggregateElement(const TermContainer &terms, const LiteralContainer &conditions) : terms(terms),
                                                                                     conditions(conditions) {};

  explicit AggregateElement(const TermContainer &terms) : terms(terms), conditions() {};

  explicit AggregateElement(const AspTerm &term) : terms(term_to_container(term)), conditions() {};
};
typedef std::vector<AggregateElement> AggregateElementList;

static AggregateElementList terms_to_aggregate_element_list(const TermContainer &terms) {
  AggregateElementList elements;
  for (const auto &term: terms) {
    elements.emplace_back(term);
  }
  return elements;
}


enum AggregateType {
  Count,
  Min,
  Max,
  Sum,
  SumPositive
};

/**
 * Aggregates let you form values from groups of atoms. They're often used alone in the head to form a "choice".
 */
struct AspAggregate: public AspElement {
  const AggregateRelation lower_relation;
  const AggregateRelation upper_relation;
  const AggregateType type;
  const std::vector<AggregateElement> elements;

  AspAggregate(const IntTerm &lower_term, TermContainer terms, const IntTerm &upper_term) : type(Count),
                                                                                            lower_relation(LessEqual,
                                                                                                           lower_term),
                                                                                            elements(
                                                                                                terms_to_aggregate_element_list(
                                                                                                    terms)),
                                                                                            upper_relation(LessEqual,
                                                                                                           upper_term) {}

  AspAggregate(const IntTerm &lower_term, AggregateElementList elements, const IntTerm &upper_term): type(Count),
                                                                                                     lower_relation(LessEqual, lower_term), elements(elements), upper_relation(LessEqual, upper_term){}

  AspAggregate(AspTerm &lower_term, AggregateElementList elements, AspTerm &upper_term): type(Count),
                                                                                         lower_relation(LessEqual, lower_term), elements(elements), upper_relation(LessEqual, upper_term){}


};

//TODO: Aggregate rules. Aggregates in the body
}