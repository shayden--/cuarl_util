#include "../src/pff_run_functions.hpp"

#include <gtest/gtest.h>

TEST(inputValidation,fileExists_pass)
{
  ASSERT_TRUE(fileExists("dovetrainlongman.dll"));
}

TEST(inputValidation,fileExists_fail)
{
  ASSERT_FALSE(fileExists("dovetrainlongman.dll"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
