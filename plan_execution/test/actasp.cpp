#include <iostream>
#include <string>
#include <actasp/reasoners/Clingo.h>
#include <gtest/gtest.h>
#include <ros/package.h>

using std::vector;
using std::string;
using std::cout;
using std::endl;
using namespace actasp;
class ClingoTest : public ::testing::Test {
protected:

  ClingoTest() {
    std::set<AspFluent, ActionComparator> actions = {"bit_on()"_f, "bit_off()"_f};
    query_generator = std::unique_ptr<FilteringQueryGenerator>(Clingo::getQueryGenerator("n", ros::package::getPath("plan_execution")+"/test/domain/", {}, actions));

  }

  void SetUp() override {

  }

  std::unique_ptr<FilteringQueryGenerator> query_generator;
};

TEST(AspFluent, InvalidConstructionThrows) {
  EXPECT_ANY_THROW(AspFluent("missing_parens"));
  //EXPECT_ANY_THROW(AspFluent("missing_time_param()"));
}


TEST(AspFluent, LiteralConstructionWorks) {

  auto fluent = "fluent(1, 2)"_f;
  EXPECT_EQ(fluent.getName(), "fluent");
  EXPECT_EQ(fluent.getTimeStep(), 2);
  EXPECT_EQ(fluent.getParameters().size(), 1);
  EXPECT_EQ(fluent.getParameters()[0], "1");
  //EXPECT_ANY_THROW(AspFluent("missing_time_param()"));
}


TEST(AspRule, EqualityWorks) {
  AspRule empty;
  EXPECT_TRUE(empty == empty);
  EXPECT_EQ("fluent(1, 2)"_f, "fluent(1, 2)"_f);
}

TEST_F(ClingoTest, MinimalPlanQueryWorks) {
  std::vector<AspRule> goal = {AspRule({},{"not bit_on(1,n)"_f})};
  auto plan = query_generator->minimalPlanQuery(goal, false,2,0);
  EXPECT_TRUE(plan.size() > 0);
}


// Run all the tests
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
