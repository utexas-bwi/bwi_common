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
    query_generator = std::unique_ptr<FilteringQueryGenerator>(Clingo::getQueryGenerator("n", ros::package::getPath("plan_execution")+"/test/domain/", {}, {"bit_on", "bit_off"}));

  }

  void SetUp() override {
  }

  std::unique_ptr<FilteringQueryGenerator> query_generator;
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
  EXPECT_EQ(boost::get<int>(atom.getParameters()[0]), 1);

  atom = AspAtom("test(1,2,3)");
  EXPECT_EQ(atom.getName(), "test");
  EXPECT_EQ(atom.arity(), 3);
  EXPECT_EQ(boost::get<int>(atom.getParameters()[0]), 1);
  EXPECT_EQ(boost::get<int>(atom.getParameters()[1]), 2);
  EXPECT_EQ(boost::get<int>(atom.getParameters()[2]), 3);

  atom = AspAtom("name('test', 2)");
  EXPECT_EQ(atom.getName(), "name");
  EXPECT_EQ(atom.arity(), 2);
  EXPECT_EQ(boost::get<std::string>(atom.getParameters()[0]), "test");
  EXPECT_EQ(boost::get<int>(atom.getParameters()[1]), 2);
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
  const auto as_string = "this_is_a_test(\"string var\", 1234, \"another var\")";
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
  EXPECT_EQ(boost::get<std::string>(fluent.getParameters()[0]), "test");
  EXPECT_EQ(boost::get<int>(fluent.getParameters()[1]), 2);
  //EXPECT_ANY_THROW(AspFluent("missing_time_param()"));
}

TEST(AspFluent, LiteralConstructionWorks) {
  auto fluent = "name('test', 2)"_f;
  EXPECT_EQ(fluent.getName(), "name");
  EXPECT_EQ(fluent.getTimeStep(), 2);
  EXPECT_EQ(fluent.getParameters().size(), 2);
  EXPECT_EQ(boost::get<std::string>(fluent.getParameters()[0]), "test");
  EXPECT_EQ(boost::get<int>(fluent.getParameters()[1]), 2);
  //EXPECT_ANY_THROW(AspFluent("missing_time_param()"));
}

TEST(AspFluent, ConstructionWorks) {
  auto fluent = AspFluent("name", {"test", 2});
  EXPECT_EQ(fluent.getTimeStep(), 2);
  EXPECT_EQ(fluent.getParameters().size(), 2);
  EXPECT_EQ(boost::get<std::string>(fluent.getParameters()[0]), "test");
  EXPECT_EQ(boost::get<int>(fluent.getParameters()[1]), 2);
}

TEST(AspFluent, EqualityWorks) {
  EXPECT_EQ("fluent(1, 2)"_f, "fluent(1, 2)"_f);
}


TEST(AspRule, EqualityWorks) {
  AspRule empty;
  EXPECT_TRUE(empty == empty);
}

TEST_F(ClingoTest, MinimalPlanQueryWorks) {
  std::vector<AspRule> goal = {AspRule({},{"not bit_on(1,n)"_f})};
  auto plan = query_generator->minimalPlanQuery(goal, false,2,0);
  EXPECT_TRUE(!plan.empty());
}

// Run all the tests
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
