#include <roboteam_utils/Hungarian.h>
#include <gtest/gtest.h>

// test a normal square vector
TEST(HungarianTest, base_test) {
  vector<vector<double>> EXAMPLE2 =
      {{46, 15, 46, 97, 31},
       {32, 73, 1,  93, 57},
       {28, 92, 9,  17, 72},
       {21, 63, 50, 44, 88},
       {50, 40, 1,  65, 2}};

  std::vector<int> assignments;
  auto cost = rtt::Hungarian::Solve(EXAMPLE2, assignments);
  std::vector<int> expectedSolution = {1, 2, 3, 0, 4};
  EXPECT_EQ(assignments, expectedSolution);
  EXPECT_EQ(cost, 56.0);
}

// this vector is non-square, so a column -1 should be appended
TEST(HungarianTest, non_square_matrix) {
  vector<vector<double>> EXAMPLE1 = {
      {100, 100,    1},
      {100, 2,      21512},
      {1,   4,      9852},
      {6,   30252,  400}};
  std::vector<int> assignments;
  double cost = rtt::Hungarian::Solve(EXAMPLE1, assignments);
  std::vector<int> expectedSolution = {2, 1, 0, -1};

  EXPECT_EQ(assignments, expectedSolution);
  EXPECT_EQ(cost, 4.0);
}

// test the helper function that we can succesfully identify a pair by a unique value.
TEST(HungarianTest, helper_identifier) {
  std::unordered_map<int, rtt::Vector2> identifiedLocations;
  identifiedLocations.insert({0, {0,0}});
  identifiedLocations.insert({1, {2,2}});
  identifiedLocations.insert({2, {4,4}});

  std::vector<rtt::Vector2> targetLocations = {
      {2.2,2.4},
      {1.1,0.4},
      {5.2,6.4},
  };

  auto result = rtt::Hungarian::getOptimalPairsIdentified(identifiedLocations, targetLocations);

  EXPECT_EQ(result.at(0), targetLocations.at(1));
  EXPECT_EQ(result.at(1), targetLocations.at(0));
  EXPECT_EQ(result.at(2), targetLocations.at(2));
}