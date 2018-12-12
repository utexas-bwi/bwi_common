#include <knowledge_representation/LongTermMemoryConduitInterface.h>
#include <iostream>
#include <string>
#include <knowledge_representation/Compiler.h>

#include <gtest/gtest.h>


using std::vector;
using std::string;
using std::cout;
using std::endl;
using knowledge_rep::Compiler;

class ParserTest : public ::testing::Test {
protected:

  ParserTest() : compiler(){}

  void SetUp() override {

  }

  knowledge_rep::Compiler compiler;
};

TEST_F(ParserTest, CompilerWorks) {
  compiler.parse_string("? dummy 1");
  ASSERT_NE(nullptr, compiler.get_root());
}

TEST_F(ParserTest, EmptyProgramParses) {
  compiler.parse_string("");
}


// Run all the tests
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
