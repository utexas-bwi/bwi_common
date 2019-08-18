#include <iostream>
#include <string>
#include <actasp/reasoners/Clingo.h>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <actasp/action_utils.h>

using std::vector;
using std::string;
using std::cout;
using std::endl;
using namespace actasp;

class SolverTest : public ::testing::Test {
protected:

  SolverTest() : query_generator(std::unique_ptr<Solver>(
      actasp::Clingo::getQueryGenerator(ros::package::getPath("plan_execution") + "/test/domain/"))){
  }

  void SetUp() override {
  }

  std::unique_ptr<Solver> query_generator;

};

TEST_F(SolverTest, MinimalPlanQueryWorks) {
  vector<AspFluent> test = {"bit_on(1,n)"_f};
  EXPECT_EQ(test[0].arity(), 2);
  std::vector<AspRule> goal = make_goal_all_true({"bit_on(1,n)"_f});
  auto plan = query_generator->minimalPlanQuery(goal, 10, 0, nullptr);
  EXPECT_TRUE(!plan.empty());

  goal = {make_goal_all_true({"bit_on(1,n)"_f, "-bit_on(1,n)"_f})};
  plan = query_generator->minimalPlanQuery(goal, 2, 0, nullptr);
  EXPECT_TRUE(plan.empty());
}

TEST_F(SolverTest, MonitorQueryWorks) {
  // Turn on all the bits... without using the helpful all_on action :-P
  std::vector<AspRule> goal{
      AspRule::integrity_constraint({"not bit_on(I, n)"_al, "query(n)"_al, "bit(I)"_al}),
      AspRule::integrity_constraint("all_on(1..n)"_al, "query(n)"_al)
  };
  auto plans = query_generator->minimalPlanQuery(goal, 2, 0, nullptr);
  // This part has to work for us to be able to test monitoring...
  ASSERT_FALSE(plans.empty());
  // 2 ways to accomplish the goal
  ASSERT_EQ(plans.size(), 2);
  auto plan = plans.front();
  ASSERT_EQ(plan.actions.size(), 2);

  auto monitor_results = query_generator->monitorQuery(goal, plan, nullptr);
  EXPECT_FALSE(monitor_results.empty());
  // Original plan is valid. No other plan should come back.
  EXPECT_EQ(monitor_results.size(), 1);
  EXPECT_EQ(plan.actions, monitor_results.front().actions);

  // Suddenly, we discover there's a third bit! (Domain defaults to 2)
  // The previous plan should no longer work
  vector<AspFact> knowledge = {AspFact::fact("bit(3)"_a), AspFact::fact("-bit_on(3,0)"_a)};
  monitor_results = query_generator->monitorQuery(goal, plan, &knowledge);
  EXPECT_TRUE(monitor_results.empty());
}

TEST_F(SolverTest, LengthRangePlanQueryWorks) {
  std::vector<AspRule> goal = make_goal_all_true({"bit_on(1,n)"_f});
  auto plans = query_generator->lengthRangePlanQuery(goal, 3, 3, 0, nullptr);
  ASSERT_FALSE(plans.empty());
  // We asked for plans exactly three steps long
  for (const auto &plan: plans) {
    EXPECT_EQ(plan.actions.size(), 3);
  }
}

TEST_F(SolverTest, OptimalPlanQueryWorks) {
  std::vector<AspRule> goal = make_goal_all_true({"bit_on(1,n)"_f, "bit_on(2,n)"_f});
  auto plan = query_generator->optimalPlanQuery(goal, 10, 0, nullptr);
  EXPECT_TRUE(plan.satisfied);
  EXPECT_EQ(plan.maxTimeStep(), 1);
}

TEST_F(SolverTest, GetAllActionsWorks) {
  std::set<std::string> all_actions = {"all_off","all_on", "turn_off", "turn_on"};
  EXPECT_EQ(query_generator->get_all_actions(), all_actions);
}


