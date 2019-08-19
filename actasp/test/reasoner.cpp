#include <iostream>
#include <string>
#include <actasp/reasoners/Clingo.h>
#include <actasp/Solver.h>
#include <gtest/gtest.h>
#include <actasp/action_utils.h>
#include <actasp/reasoners/Reasoner.h>

using std::vector;
using std::string;
using std::cout;
using std::endl;
using namespace actasp;

class ReasonerTest : public ::testing::Test {
protected:

  ReasonerTest() : solver(std::unique_ptr<Solver>(
      actasp::Clingo::getSolver(std::getenv("DOMAIN_PATH")))),
                   reasoner(solver.get(), 10, solver->get_all_actions()) {
  }

  void SetUp() override {
  }

  std::unique_ptr<Solver> solver;
  Reasoner reasoner;
};


TEST_F(ReasonerTest, AvailableActionsWorks) {
  std::set<AspFluent> all_actions = {"all_off(1)"_f, "all_on(1)"_f, "turn_off(1,1)"_f, "turn_off(2,1)"_f,
                                     "turn_on(1,1)"_f, "turn_on(2,1)"_f};
  auto actions = reasoner.availableActions();
  EXPECT_EQ(actions, all_actions);
}


TEST_F(ReasonerTest, ComputePolicyWorks) {
  std::vector<AspRule> goal = make_goal_all_true({"bit_on(1,n)"_f, "bit_on(2,n)"_f});
  auto actions = reasoner.computePolicy(goal, 1.0);
  actions = reasoner.computePolicy(goal, 2.0);

  goal = std::vector<AspRule>{
      AspRule::integrity_constraint({"not bit_on(I, n)"_al, "query(n)"_al, "bit(I)"_al}),
      AspRule::integrity_constraint("all_on(1..n)"_al, "query(n)"_al)
  };

  actions = reasoner.computePolicy(goal, 1.0);
}




