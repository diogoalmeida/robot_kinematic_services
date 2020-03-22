#include <gtest/gtest.h>
#include <ros/ros.h>

class TestKinematicServices : public ::testing::Test
{
 public:
  void init(int argc, char** argv)
  {
    argc_ = argc;
    argv_ = argv;
  }

 protected:
  void SetUp() override { ros::init(argc_, argv_, "my_test_node"); }

  int argc_;
  char** argv_;
};

TEST(TestKinematicServices, TestParameterLoading) { EXPECT_TRUE(true); }

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}