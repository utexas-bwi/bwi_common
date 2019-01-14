#include <iostream>
#include <string>
#include <actasp/reasoners/Clingo.h>
#include <actasp/reasoners/FilteringReasoner.h>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <actasp/action_utils.h>

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

TEST(AspAtom, InvalidConstructionThrows) {
  EXPECT_ANY_THROW(AspFluent("missing_parens"));
  //EXPECT_ANY_THROW(AspFluent("missing_time_param()"));
}

TEST(AspAtom, StringConstructionWorks) {
  auto atom = AspAtom("test()");
  EXPECT_EQ(atom.getName(), "test");
  EXPECT_EQ(atom.arity(), 0);

  atom = AspAtom("test(1)");
  EXPECT_EQ(atom.getName(), "test");
  EXPECT_EQ(boost::get<int>(atom.getArguments()[0]), 1);

  atom = AspAtom("test(1,2,3)");
  EXPECT_EQ(atom.getName(), "test");
  EXPECT_EQ(atom.arity(), 3);
  EXPECT_EQ(boost::get<int>(atom.getArguments()[0]), 1);
  EXPECT_EQ(boost::get<int>(atom.getArguments()[1]), 2);
  EXPECT_EQ(boost::get<int>(atom.getArguments()[2]), 3);

  atom = AspAtom("name('test', 2)");
  EXPECT_EQ(atom.getName(), "name");
  EXPECT_EQ(atom.arity(), 2);
  EXPECT_EQ(boost::get<std::string>(atom.getArguments()[0]), "test");
  EXPECT_EQ(boost::get<int>(atom.getArguments()[1]), 2);

  atom = AspAtom("not test()");
  EXPECT_EQ(atom.getName(), "test");
  EXPECT_EQ(atom.getNegation().size(), 1);
  EXPECT_EQ(atom.getNegation()[0], Negation::Default);

  atom = AspAtom("not not --test()");
  EXPECT_EQ(atom.getNegation().size(), 4);
  EXPECT_EQ(atom.getNegation()[0], Negation::Default);
  EXPECT_EQ(atom.getNegation()[1], Negation::Default);
  EXPECT_EQ(atom.getNegation()[2], Negation::Classical);
  EXPECT_EQ(atom.getNegation()[3], Negation::Classical);
}

TEST(AspAtom, ComparisonWorks) {
  auto atom = AspAtom("test()");
  EXPECT_EQ(atom, atom);
  EXPECT_FALSE(atom < atom);
  EXPECT_FALSE(atom > atom);
  auto first_by_arg = AspAtom("test(1)");
  auto first_by_name = AspAtom("aa()");
  auto mid_by_second_arg = AspAtom("test(1, 2)");
  auto last_by_arg = AspAtom("test(2)");
  EXPECT_LT(first_by_name, first_by_arg);
  EXPECT_LT(first_by_name, last_by_arg);
  EXPECT_LT(first_by_name, mid_by_second_arg);

  EXPECT_LT(first_by_arg, last_by_arg);
  EXPECT_LT(first_by_arg, mid_by_second_arg);

  EXPECT_LT(mid_by_second_arg, last_by_arg);

}

TEST(AspAtom, ToStringWorks) {
  const auto as_string = "not -this_is_a_test(\"string var\", 1234, \"another var\")";
  const auto atom = AspAtom(as_string);
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
  EXPECT_EQ(boost::get<std::string>(fluent.getArguments()[0]), "test");
  EXPECT_EQ(boost::get<int>(fluent.getArguments()[1]), 2);
  //EXPECT_ANY_THROW(AspFluent("missing_time_param()"));
}

TEST(AspFluent, LiteralConstructionWorks) {
  auto fluent = "name('test', 2)"_f;
  EXPECT_EQ(fluent.getName(), "name");
  EXPECT_EQ(fluent.getTimeStep(), 2);
  EXPECT_EQ(fluent.getArguments().size(), 2);
  EXPECT_EQ(boost::get<std::string>(fluent.getArguments()[0]), "test");
  EXPECT_EQ(boost::get<int>(fluent.getArguments()[1]), 2);
  //EXPECT_ANY_THROW(AspFluent("missing_time_param()"));
}

TEST(AspFluent, ConstructionWorks) {
  auto fluent = AspFluent("name", {"test", 2});
  EXPECT_EQ(fluent.getTimeStep(), 2);
  EXPECT_EQ(fluent.getArguments().size(), 2);
  EXPECT_EQ(boost::get<std::string>(fluent.getArguments()[0]), "test");
  EXPECT_EQ(boost::get<int>(fluent.getArguments()[1]), 2);
}

TEST(AspFluent, EqualityWorks) {
  EXPECT_EQ("fluent(1, 2)"_f, "fluent(1, 2)"_f);
}

TEST(AspRule, EqualityWorks) {
  AspFluentRule empty;
  EXPECT_TRUE(empty == empty);
}

TEST_F(ReasonerTest, MinimalPlanQueryWorks) {
  std::vector<AspFluentRule> goal = make_goal_all_true({"bit_on(1,n)"_f});
  auto plan = query_generator->minimalPlanQuery(goal,10,0);
  EXPECT_TRUE(!plan.empty());

  goal = {make_goal_all_true({"bit_on(1,n)"_f, "-bit_on(1,n)"_f})};
  plan = query_generator->minimalPlanQuery(goal,2,0);
  EXPECT_TRUE(plan.empty());
}

TEST_F(ReasonerTest, LengthRangePlanQueryWorks) {
  std::vector<AspFluentRule> goal = make_goal_all_true({"bit_on(1,n)"_f});
  auto plan = query_generator->lengthRangePlanQuery(goal,10,10,0);
  EXPECT_TRUE(!plan.empty());
}

TEST_F(ReasonerTest, OptimalPlanQueryWorks) {
  std::vector<AspFluentRule> goal = make_goal_all_true({"bit_on(1,n)"_f, "bit_on(2,n)"_f});

  auto plan = query_generator->optimalPlanQuery(goal,10,0);
  EXPECT_TRUE(plan.isSatisfied());
  EXPECT_EQ(plan.maxTimeStep(), 1);
}

TEST_F(ReasonerTest, GetAllActionsWorks) {
  std::set<std::string> all_actions = {"all_off","all_on", "turn_off", "turn_on"};
  EXPECT_EQ(query_generator->get_all_actions(), all_actions);
}

TEST_F(ReasonerTest, AvailableActionsWorks){
  std::set<AspFluent> all_actions = {"all_off(0)"_f,"all_on(0)"_f, "turn_off(1,0)"_f, "turn_off(2,0)"_f "turn_on(1,0)"_f, "turn_off(2,0)"_f};
  auto actions = reasoner.availableActions();
  EXPECT_EQ(actions, all_actions);
}

// Run all the tests
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
