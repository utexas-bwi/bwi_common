#include <iostream>
#include <string>
#include <actasp/reasoners/Clingo.h>
#include <actasp/reasoners/FilteringReasoner.h>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <actasp/action_utils.h>
#include <actasp/asp/AspTerm.h>
#include <actasp/asp/AspAggregate.h>

using std::vector;
using std::string;
using std::cout;
using std::endl;
using namespace actasp;
class ReasonerTest : public ::testing::Test {
protected:

  ReasonerTest(): query_generator(std::unique_ptr<FilteringQueryGenerator>(Clingo::getQueryGenerator(ros::package::getPath("plan_execution")+"/test/domain/"))),
  reasoner(query_generator.get(),10, query_generator->get_all_actions()) {
  }

  void SetUp() override {
  }

  std::unique_ptr<FilteringQueryGenerator> query_generator;
  FilteringReasoner reasoner;
};

TEST(AspFunction, InvalidConstructionThrows) {
  EXPECT_ANY_THROW(AspFluent("missing_parens"));
  //EXPECT_ANY_THROW(AspFluent("missing_time_param()"));
}

TEST(AspFunction, StringConstructionWorks) {
  auto atom = AspFunction("test()");
  EXPECT_EQ(atom.getName(), "test");
  EXPECT_EQ(atom.arity(), 0);

  atom = AspFunction("test(1)");
  EXPECT_EQ(atom.getName(), "test");
  EXPECT_EQ(SimpleTerm::get_int(atom.getArguments()[0]), 1);

  atom = AspFunction("test(1,2,3)");
  EXPECT_EQ(atom.getName(), "test");
  EXPECT_EQ(atom.arity(), 3);
  EXPECT_EQ(SimpleTerm::get_int(atom.getArguments()[0]), 1);
  EXPECT_EQ(SimpleTerm::get_int(atom.getArguments()[1]), 2);
  EXPECT_EQ(SimpleTerm::get_int(atom.getArguments()[2]), 3);

  atom = AspFunction("name('test', 2)");
  EXPECT_EQ(atom.getName(), "name");
  EXPECT_EQ(atom.arity(), 2);
  EXPECT_EQ(SimpleTerm::get_string(atom.getArguments()[0]), "test");
  EXPECT_EQ(SimpleTerm::get_int(atom.getArguments()[1]), 2);

  atom = AspFunction("not test()");
  EXPECT_EQ(atom.getName(), "test");
  EXPECT_EQ(atom.getNegation().size(), 1);
  EXPECT_EQ(atom.getNegation()[0], Negation::Default);

  atom = AspFunction("not not --test()");
  EXPECT_EQ(atom.getNegation().size(), 4);
  EXPECT_EQ(atom.getNegation()[0], Negation::Default);
  EXPECT_EQ(atom.getNegation()[1], Negation::Default);
  EXPECT_EQ(atom.getNegation()[2], Negation::Classical);
  EXPECT_EQ(atom.getNegation()[3], Negation::Classical);
}

TEST(AspFunction, ComparisonWorks) {
  auto atom = AspFunction("test()");
  EXPECT_EQ(atom, atom);
  EXPECT_FALSE(atom < atom);
  EXPECT_FALSE(atom > atom);
  auto first_by_arg = AspFunction("test(1)");
  auto first_by_name = AspFunction("aa()");
  auto mid_by_second_arg = AspFunction("test(1, 2)");
  auto last_by_arg = AspFunction("test(2)");
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
  const auto as_string = "not -this_is_a_test(\"string var\", 1234, \"another var\")";
  const auto atom = AspFunction(as_string);
  EXPECT_EQ(as_string, (std::string)atom);
}

TEST(AspFluent, InvalidConstructionThrows) {
  EXPECT_ANY_THROW(AspFluent("missing_parens"));
  //EXPECT_ANY_THROW(AspFluent("missing_time_param()"));
}



TEST(AspFluent, StringConstructionWorks) {
  auto fluent = AspFluent("name('test', 2)");
  EXPECT_EQ(fluent.getName(), "name");
  EXPECT_EQ(fluent.getTimeStep(), 2);
  EXPECT_EQ(fluent.arity(), 2);
  EXPECT_EQ(SimpleTerm::get_string(fluent.getArguments()[0]), "test");
  EXPECT_EQ(SimpleTerm::get_int(fluent.getArguments()[1]), 2);
  //EXPECT_ANY_THROW(AspFluent("missing_time_param()"));
}

TEST(AspFluent, LiteralConstructionWorks) {
  auto fluent = "name('test', 2)"_f;
  EXPECT_EQ(fluent.getName(), "name");
  EXPECT_EQ(fluent.getTimeStep(), 2);
  EXPECT_EQ(fluent.getArguments().size(), 2);
  EXPECT_EQ(SimpleTerm::get_string(fluent.getArguments()[0]), "test");
  EXPECT_EQ(SimpleTerm::get_int(fluent.getArguments()[1]), 2);
  //EXPECT_ANY_THROW(AspFluent("missing_time_param()"));
}

TEST(AspFluent, ConstructionWorks) {
  TermContainer terms;
  terms.push_back(new StringTerm("test"));
  terms.push_back(new IntTerm(2));
  auto fluent = AspFluent("name", terms);
  EXPECT_EQ(fluent.getTimeStep(), 2);
  EXPECT_EQ(fluent.getArguments().size(), 2);
  EXPECT_EQ(SimpleTerm::get_string(fluent.getArguments()[0]), "test");
  EXPECT_EQ(SimpleTerm::get_int(fluent.getArguments()[1]), 2);
}

TEST(AspFluent, AllConstructionEquivalent) {
  const auto literal = "-name('test',2)"_f;
  TermContainer terms;
  terms.push_back(new StringTerm("test"));
  terms.push_back(new IntTerm(2));
  const auto plain = AspFluent("name", terms, {Negation::Classical});
  const auto string = AspFluent("-name('test',2)");
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
  terms.push_back(new AspFluent("test(blah, 2)"));
  terms.push_back(new AspFunction("another(thing)"));

  TermContainer cloned = terms;
  EXPECT_EQ(terms, cloned);
}

TEST(AspAggregate, ConstructionWorks) {
  AspAggregate aggregate(IntTerm(0), {{}}, IntTerm(1));
}