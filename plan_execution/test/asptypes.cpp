#include <iostream>
#include <string>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <actasp/action_utils.h>
#include <actasp/asp/AspTerm.h>
#include <actasp/asp/AspAggregate.h>

using namespace std;
using namespace actasp;

class AspTypesTest : public ::testing::Test {
protected:

  AspTypesTest() {

  }

  void SetUp() override {
  }

};

TEST(AspFunction, InvalidConstructionThrows) {
  EXPECT_ANY_THROW(AspFunction::from_string("not this_is_a_literal()"));
  EXPECT_ANY_THROW(AspFunction::from_string("blah(-)"));
  EXPECT_ANY_THROW(AspFunction::from_string("hello()()"));
  EXPECT_ANY_THROW(AspFunction::from_string("hello(N)"));
}

TEST(AspFunction, StringConstructionWorks) {
  auto atom = AspFunction::from_string("test()");
  EXPECT_EQ(atom.name, "test");
  EXPECT_EQ(atom.arity(), 0);

  atom = AspFunction::from_string("test(1)");
  EXPECT_EQ(atom.name, "test");
  EXPECT_EQ(SimpleTerm::get_int(atom.arguments[0]), 1);

  atom = AspFunction::from_string("test(1,2,3)");
  EXPECT_EQ(atom.name, "test");
  EXPECT_EQ(atom.arity(), 3);
  EXPECT_EQ(SimpleTerm::get_int(atom.arguments[0]), 1);
  EXPECT_EQ(SimpleTerm::get_int(atom.arguments[1]), 2);
  EXPECT_EQ(SimpleTerm::get_int(atom.arguments[2]), 3);

  atom = AspFunction::from_string("name('test', 2)");
  EXPECT_EQ(atom.name, "name");
  EXPECT_EQ(atom.arity(), 2);
  EXPECT_EQ(SimpleTerm::get_string(atom.arguments[0]), "test");
  EXPECT_EQ(SimpleTerm::get_int(atom.arguments[1]), 2);

  atom = AspFunction::from_string("name(n)");
  EXPECT_EQ(atom.name, "name");
  EXPECT_EQ(atom.arity(), 1);
  EXPECT_EQ(SimpleTerm::get_constant(atom.arguments[0]).name, "n");
}

TEST(AspFunction, ComparisonWorks) {
  auto atom = AspFunction::from_string("test()");
  EXPECT_EQ(atom, atom);
  EXPECT_FALSE(atom < atom);
  EXPECT_FALSE(atom > atom);
  auto first_by_arg = AspFunction::from_string("test(1)");
  auto first_by_name = AspFunction::from_string("aa()");
  auto mid_by_second_arg = AspFunction::from_string("test(1, 2)");
  auto last_by_arg = AspFunction::from_string("test(2)");
  EXPECT_FALSE(first_by_arg == first_by_name);
  EXPECT_FALSE(first_by_arg == mid_by_second_arg);
  EXPECT_FALSE(first_by_arg == last_by_arg);
  EXPECT_LT(first_by_name, first_by_arg);
  EXPECT_LT(first_by_name, last_by_arg);
  EXPECT_LT(first_by_name, mid_by_second_arg);

  EXPECT_LT(first_by_arg, last_by_arg);
  EXPECT_LT(first_by_arg, mid_by_second_arg);

  EXPECT_LT(mid_by_second_arg, last_by_arg);

}

TEST(AspFunction, ToStringWorks) {
  const auto as_string = "-this_is_a_test(\"string var\", 1234, \"another var\")";
  const auto atom = AspFunction::from_string(as_string);
  EXPECT_EQ(as_string, (std::string)atom);
}

TEST(AspFluent, InvalidConstructionThrows) {
  EXPECT_ANY_THROW(AspFluent::from_string("missing_parens"));
  //EXPECT_ANY_THROW(AspFluent("missing_time_param()"));
}

TEST(AspFluent, StringConstructionWorks) {
  auto fluent = AspFluent::from_string("name('test', 2)");
  EXPECT_EQ(fluent.name, "name");
  EXPECT_EQ(fluent.getTimeStep(), 2);
  EXPECT_EQ(fluent.arity(), 2);
  EXPECT_EQ(SimpleTerm::get_string(fluent.arguments[0]), "test");
  EXPECT_EQ(SimpleTerm::get_int(fluent.arguments[1]), 2);
  //EXPECT_ANY_THROW(AspFluent("missing_time_param()"));
}

TEST(AspFluent, StringLiteralConstructionWorks) {
  auto fluent = "name('test', 2)"_f;
  EXPECT_EQ(fluent.name, "name");
  EXPECT_EQ(fluent.getTimeStep(), 2);
  EXPECT_EQ(fluent.arguments.size(), 2);
  EXPECT_EQ(SimpleTerm::get_string(fluent.arguments[0]), "test");
  EXPECT_EQ(SimpleTerm::get_int(fluent.arguments[1]), 2);
  //EXPECT_ANY_THROW(AspFluent("missing_time_param()"));
}

TEST(AspFluent, ConstructionWorks) {
  TermContainer terms;
  terms.push_back(new StringTerm("test"));
  terms.push_back(new IntTerm(2));
  auto fluent = AspFluent("name", terms);
  EXPECT_EQ(fluent.getTimeStep(), 2);
  EXPECT_EQ(fluent.arguments.size(), 2);
  EXPECT_EQ(SimpleTerm::get_string(fluent.arguments[0]), "test");
  EXPECT_EQ(SimpleTerm::get_int(fluent.arguments[1]), 2);
}

TEST(AspFluent, AllConstructionEquivalent) {
  const auto literal = "-name('test',2)"_f;
  TermContainer terms;
  terms.push_back(new StringTerm("test"));
  terms.push_back(new IntTerm(2));
  const auto plain = AspFluent("name", terms, true);
  const auto string = AspFluent::from_string("-name('test',2)");
  EXPECT_EQ(literal, plain);
  EXPECT_EQ(string, plain);
  EXPECT_EQ(string, literal);
}

TEST(AspFluent, EqualityWorks) {
  const auto first = "fluent(1, 2)"_f;
  const auto second = "fluent(1, 2)"_f;
  EXPECT_EQ(first, second);
  EXPECT_FALSE("fluent('hi',1)"_f == "fluent('hello',2)"_f);
}

TEST(AspFluent, CopyWorks) {
  const auto fluent = "bit_on(1,n)"_f;
  vector<AspFluent> test = {fluent};
  EXPECT_EQ(fluent, test[0]);
  vector<AspFluent> second = {"bit_on(1,n)"_f};
  EXPECT_EQ(test, second);
}

TEST(AspLiteral, StringConstructionWorks) {
  {
    AtomLiteral literal = AtomLiteral::from_string("test()");
    EXPECT_EQ(literal.negation, NegationType::None);
    EXPECT_EQ(literal.atom.name, "test");
    EXPECT_EQ(literal.atom.negative, false);
    EXPECT_EQ(literal.atom.arity(), 0);
  }
  {
    AtomLiteral literal = AtomLiteral::from_string("-test()");
    EXPECT_EQ(literal.negation, NegationType::None);
    EXPECT_EQ(literal.atom.name, "test");
    EXPECT_EQ(literal.atom.negative, true);
    EXPECT_EQ(literal.atom.arity(), 0);
  }
  {
    AtomLiteral literal = AtomLiteral::from_string("not test()");
    EXPECT_EQ(literal.negation, NegationType::Negation);
    EXPECT_EQ(literal.atom.name, "test");
    EXPECT_EQ(literal.atom.negative, false);
    EXPECT_EQ(literal.atom.arity(), 0);
  }
  {
    AtomLiteral literal = AtomLiteral::from_string("not not test()");
    EXPECT_EQ(literal.negation, NegationType::DoubleNegation);
    EXPECT_EQ(literal.atom.negative, false);
    EXPECT_EQ(literal.atom.arity(), 0);
  }
  {
    AtomLiteral literal = AtomLiteral::from_string("-test()");
    EXPECT_EQ(literal.negation, NegationType::None);
    EXPECT_EQ(literal.atom.name, "test");
    EXPECT_EQ(literal.atom.negative, true);
    EXPECT_EQ(literal.atom.arity(), 0);
  }
  {
    AtomLiteral literal = AtomLiteral::from_string("not not -test('string', 1234, symbolic_constant, Variable)");
    EXPECT_EQ(literal.negation, NegationType::DoubleNegation);
    EXPECT_EQ(literal.atom.name, "test");
    EXPECT_EQ(literal.atom.negative, true);
    EXPECT_EQ(literal.atom.arity(), 4);
  }
  {
    AtomLiteral literal = AtomLiteral::from_string("test(1..pool)");
    EXPECT_EQ(literal.atom.name, "test");
    EXPECT_EQ(literal.atom.arity(), 1);
  }
}

TEST(AspRule, EqualityWorks) {
  AspRule empty;
  EXPECT_TRUE(empty == empty);
  const vector<actasp::AspAtom> body = {AspFunction("hi",{})};
  const vector<actasp::AspAtom> head = {AspFunction("hi", {})};
  AspRule with_body = AspRule::with_atoms({}, body);
  AspRule with_head = AspRule::with_atoms(head, {});
  AspRule full_rule = AspRule::with_atoms(head, body);
  EXPECT_FALSE(empty == with_body);
  EXPECT_FALSE(empty == with_head);
  EXPECT_FALSE(empty == full_rule);
  EXPECT_FALSE(with_body == with_head);
  EXPECT_EQ(full_rule, full_rule);
}

TEST(AspRule, ConvenienceConstructionWorks) {
  auto constraint = AspRule::integrity_constraint({"not bit_on(I, n)"_al, "query(n)"_al, "bit(I)"_al});
  EXPECT_EQ(constraint.head.size(), 0);
  EXPECT_EQ(constraint.body.size(), 3);
  auto fact = AspFact::fact("query(n)"_al);
  EXPECT_EQ(fact.body.size(), 0);
  EXPECT_EQ(fact.head.size(), 1);

}

TEST(AspTerm, TermContainerWorks) {
  TermContainer terms;
  terms.push_back(new IntTerm(1));
  terms.push_back(new StringTerm("Hi"));
  terms.push_back(new Variable("X"));
  terms.push_back(new SymbolicConstant("n"));
  EXPECT_EQ(SimpleTerm::get_int(terms[0]), 1);
  EXPECT_EQ(SimpleTerm::get_string(terms[1]), "Hi");
  EXPECT_EQ(SimpleTerm::get_variable(terms[2]), Variable("X"));
  EXPECT_EQ(SimpleTerm::get_constant(terms[3]), SymbolicConstant("n"));
}

TEST(AspTerm, CloneWorks) {
  TermContainer terms;
  terms.push_back(new IntTerm(1));
  terms.push_back(new StringTerm("Hi"));
  terms.push_back(new Variable("X"));
  terms.push_back(new SymbolicConstant("n"));
  terms.push_back(new AspFluent(AspFluent::from_string("test(blah, 2)")));
  terms.push_back(new AspFunction(AspFunction::from_string("another(thing)")));

  TermContainer cloned = terms;
  EXPECT_EQ(terms, cloned);
}

TEST(AspAggregate, ConstructionWorks) {
  AspAggregate aggregate(IntTerm(0), TermContainer{{}}, IntTerm(1));
}