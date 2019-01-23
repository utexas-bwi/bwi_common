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

TEST_F(ReasonerTest, MinimalPlanQueryWorks) {
  std::vector<AspRule> goal = make_goal_all_true({"bit_on(1,n)"_f});
  auto plan = query_generator->minimalPlanQuery(goal,10,0);
  EXPECT_TRUE(!plan.empty());

  goal = {make_goal_all_true({"bit_on(1,n)"_f, "-bit_on(1,n)"_f})};
  plan = query_generator->minimalPlanQuery(goal,2,0);
  EXPECT_TRUE(plan.empty());
}

TEST_F(ReasonerTest, LengthRangePlanQueryWorks) {
  std::vector<AspRule> goal = make_goal_all_true({"bit_on(1,n)"_f});
  auto plan = query_generator->lengthRangePlanQuery(goal,10,10,0);
  EXPECT_TRUE(!plan.empty());
}

TEST_F(ReasonerTest, OptimalPlanQueryWorks) {
  std::vector<AspRule> goal = make_goal_all_true({"bit_on(1,n)"_f, "bit_on(2,n)"_f});

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


