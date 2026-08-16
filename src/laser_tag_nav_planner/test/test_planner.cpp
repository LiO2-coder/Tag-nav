#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <laser_tag_nav_planner/core/astar.h>
#include <laser_tag_nav_planner/core/connectivity_loader.h>
#include <laser_tag_nav_planner/core/direction.h>
#include <laser_tag_nav_planner/core/graph.h>
#include <laser_tag_nav_planner/core/tag_map_loader.h>

namespace core = laser_tag_nav_planner::core;

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace
{

std::string writeTempFile(const std::string& name, const std::string& contents)
{
  const std::string path = "/tmp/" + name;
  std::ofstream output(path);
  output << contents;
  output.close();
  return path;
}

// 3x3 grid: id 0..8 at (x = id % 3, y = id / 3), so 0=(0,0), 8=(2,2).
core::Graph threeByThreeGraph(bool allow_diagonal)
{
  std::map<int, core::NodePose> nodes;
  std::map<int, uint8_t> connectivity;
  for (int id = 0; id < 9; ++id)
  {
    core::NodePose pose;
    pose.x = static_cast<double>(id % 3);
    pose.y = static_cast<double>(id / 3);
    nodes[id] = pose;
    connectivity[id] = 255;  // full 8-connectivity; buildGraph trims out-of-bounds edges
  }
  core::PlannerOptions options;
  options.allow_diagonal = allow_diagonal;
  return core::buildGraph(nodes, connectivity, options).graph;
}

double pathLength(const core::PathResult& path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.waypoints.size(); ++i)
    length += std::hypot(path.waypoints[i].x - path.waypoints[i - 1].x,
                         path.waypoints[i].y - path.waypoints[i - 1].y);
  return length;
}

}  // namespace

TEST(Direction, UnitStepsAndYaw)
{
  const core::Vec2d north = core::directionStep(core::Direction::N);
  EXPECT_DOUBLE_EQ(north.x, 0.0);
  EXPECT_DOUBLE_EQ(north.y, 1.0);

  EXPECT_NEAR(core::directionYaw(core::Direction::E), 0.0, 1e-12);
  EXPECT_NEAR(core::directionYaw(core::Direction::N), M_PI / 2.0, 1e-12);
  EXPECT_NEAR(core::directionYaw(core::Direction::W), M_PI, 1e-12);
  EXPECT_NEAR(core::directionYaw(core::Direction::S), -M_PI / 2.0, 1e-12);
  EXPECT_NEAR(core::directionYaw(core::Direction::NE), M_PI / 4.0, 1e-12);
}

TEST(Direction, OppositeAndWrap)
{
  EXPECT_EQ(core::opposite(core::Direction::N), core::Direction::S);
  EXPECT_EQ(core::opposite(core::Direction::NE), core::Direction::SW);
  EXPECT_EQ(core::opposite(core::Direction::E), core::Direction::W);

  EXPECT_DOUBLE_EQ(core::wrapToPi(0.0), 0.0);
  EXPECT_NEAR(core::wrapToPi(3.0 * M_PI), M_PI, 1e-12);
  EXPECT_NEAR(core::wrapToPi(2.0 * M_PI + 0.1), 0.1, 1e-12);
  EXPECT_NEAR(core::wrapToPi(-0.1), -0.1, 1e-12);
}

TEST(TagMapLoader, Parses2dLocations)
{
  const std::string path = writeTempFile("planner_test_map.json",
      R"({"map_type":"2d","tag_side_lengths":{"default":0.1},
          "tag_locations":{"0":[-9.0,12.0,0.0],"18":[9.0,12.0,0.0],"474":[9.0,-12.0,0.0]}})");
  const std::map<int, core::NodePose> nodes = core::loadTagMap(path);
  EXPECT_EQ(nodes.size(), 3u);
  EXPECT_DOUBLE_EQ(nodes.at(0).x, -9.0);
  EXPECT_DOUBLE_EQ(nodes.at(0).y, 12.0);
  EXPECT_DOUBLE_EQ(nodes.at(474).x, 9.0);
  EXPECT_DOUBLE_EQ(nodes.at(474).y, -12.0);
  std::remove(path.c_str());
}

TEST(TagMapLoader, RejectsWrongMapType)
{
  const std::string path = writeTempFile("planner_bad_map.json",
      R"({"map_type":"3d","tag_locations":{"0":[0.0,0.0,0.0]}})");
  EXPECT_THROW(core::loadTagMap(path), std::runtime_error);
  std::remove(path.c_str());
}

TEST(TagMapLoader, RejectsBadLocationDimension)
{
  const std::string path = writeTempFile("planner_bad_loc.json",
      R"({"map_type":"2d","tag_locations":{"0":[1.0,2.0]}})");
  EXPECT_THROW(core::loadTagMap(path), std::runtime_error);
  std::remove(path.c_str());
}

TEST(ConnectivityLoader, ParsesMasks)
{
  const std::string path = writeTempFile("planner_conn.json",
      R"({"schema_version":1,"connectivity":{"0":20,"255":85}})");
  const std::map<int, uint8_t> connectivity = core::loadConnectivity(path);
  EXPECT_EQ(connectivity.size(), 2u);
  EXPECT_EQ(connectivity.at(0), 20);
  EXPECT_EQ(connectivity.at(255), 85);
  std::remove(path.c_str());
}

TEST(ConnectivityLoader, RejectsOutOfRangeMask)
{
  const std::string path = writeTempFile("planner_bad_mask.json",
      R"({"connectivity":{"0":256}})");
  EXPECT_THROW(core::loadConnectivity(path), std::runtime_error);
  std::remove(path.c_str());
}

TEST(Graph, BuildsEdgesAndWarnsOnOutOfBounds)
{
  std::map<int, core::NodePose> nodes;
  nodes[0] = {0.0, 0.0, 0.0};
  nodes[1] = {1.0, 0.0, 0.0};
  nodes[2] = {0.0, 1.0, 0.0};
  std::map<int, uint8_t> connectivity{{0, 255}, {1, 255}, {2, 255}};
  core::PlannerOptions options;
  options.allow_diagonal = true;
  const core::GraphBuildResult result = core::buildGraph(nodes, connectivity, options);
  // Node 0 connects east to 1 and north to 2; the NE diagonal (1,1) is unmapped.
  EXPECT_EQ(result.graph.adjacency.at(0).size(), 2u);
  EXPECT_FALSE(result.warnings.empty());
}

TEST(Graph, DiagonalSkippedWhenDisabled)
{
  std::map<int, core::NodePose> nodes;
  nodes[0] = {0.0, 0.0, 0.0};
  nodes[1] = {1.0, 0.0, 0.0};
  nodes[2] = {0.0, 1.0, 0.0};
  nodes[3] = {1.0, 1.0, 0.0};
  std::map<int, uint8_t> connectivity{{0, 255}, {1, 255}, {2, 255}, {3, 255}};
  core::PlannerOptions options;
  options.allow_diagonal = false;
  const core::GraphBuildResult result = core::buildGraph(nodes, connectivity, options);
  EXPECT_EQ(result.graph.adjacency.at(0).size(), 2u);  // east and north only
}

TEST(AStar, StraightPath)
{
  const core::Graph graph = threeByThreeGraph(true);
  const core::PathResult path = core::planPath(graph, 0, 2, core::PlannerOptions());
  ASSERT_TRUE(path.found);
  ASSERT_EQ(path.waypoints.size(), 3u);
  EXPECT_EQ(path.waypoints[0].tag_id, 0);
  EXPECT_EQ(path.waypoints[1].tag_id, 1);
  EXPECT_EQ(path.waypoints[2].tag_id, 2);
  EXPECT_NEAR(pathLength(path), 2.0, 1e-9);
}

TEST(AStar, DiagonalPathUsesDiagonals)
{
  const core::Graph graph = threeByThreeGraph(true);
  core::PlannerOptions options;
  options.allow_diagonal = true;
  const core::PathResult path = core::planPath(graph, 0, 8, options);
  ASSERT_TRUE(path.found);
  ASSERT_EQ(path.waypoints.size(), 3u);  // 0 -> 4 -> 8
  EXPECT_EQ(path.waypoints[1].tag_id, 4);
  EXPECT_NEAR(pathLength(path), 2.0 * std::sqrt(2.0), 1e-9);
}

TEST(AStar, CardinalOnlyWithoutDiagonal)
{
  const core::Graph graph = threeByThreeGraph(false);
  core::PlannerOptions options;
  options.allow_diagonal = false;
  const core::PathResult path = core::planPath(graph, 0, 8, options);
  ASSERT_TRUE(path.found);
  ASSERT_EQ(path.waypoints.size(), 5u);
  EXPECT_NEAR(pathLength(path), 4.0, 1e-9);
}

TEST(AStar, StartEqualsGoal)
{
  const core::Graph graph = threeByThreeGraph(true);
  const core::PathResult path = core::planPath(graph, 4, 4, core::PlannerOptions());
  ASSERT_TRUE(path.found);
  ASSERT_EQ(path.waypoints.size(), 1u);
  EXPECT_EQ(path.waypoints[0].tag_id, 4);
}

TEST(AStar, Unreachable)
{
  core::Graph graph;
  graph.nodes[0] = {0.0, 0.0, 0.0};
  graph.nodes[1] = {1.0, 0.0, 0.0};
  graph.adjacency[0] = {};  // isolated start node
  EXPECT_FALSE(core::planPath(graph, 0, 1, core::PlannerOptions()).found);
}

TEST(AStar, MissingNode)
{
  const core::Graph graph = threeByThreeGraph(true);
  EXPECT_FALSE(core::planPath(graph, 0, 999, core::PlannerOptions()).found);
}
