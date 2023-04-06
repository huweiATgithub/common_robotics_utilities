#include <gtest/gtest.h>
#include <common_robotics_utilities/simple_rrt_planner.hpp>
#include "helper_functions_mainly_for_python.hpp"
#include <Eigen/Core>

namespace common_robotics_utilities::make_function_test {
    using namespace common_robotics_utilities::extras;
    using T = Eigen::VectorXd;

    static simple_rrt_planner::SimpleRRTPlannerTree<T> &getTree() {
        int num_nodes = 100;
        static simple_rrt_planner::SimpleRRTPlannerTree<T> tree(num_nodes);
        for (int i = 0; i < num_nodes; i++) {
            tree.AddNode((T(2) << i / 10.0, i / 10.0).finished());
        }
        return tree;
    }

    GTEST_TEST(ExtraTests, MakeKinematicLinearBiRRTNearestNeighborsFunctionParallel) {
        auto birrt_nearest_neighbors_fn
                = simple_rrt_planner
                ::MakeKinematicLinearBiRRTNearestNeighborsFunction<T>(
                        EuclideanDistanceFunction, true);
        T query = (T(2) << 0.0, 0.0).finished();
        const simple_rrt_planner::SimpleRRTPlannerTree<T> &tree = getTree();
        int64_t index = birrt_nearest_neighbors_fn(
                tree,
                query,
                simple_rrt_planner::BiRRTActiveTreeType::START_TREE
        );
        GTEST_ASSERT_EQ(index, 0);
    }

    GTEST_TEST(ExtraTests, MakeKinematicLinearBiRRTNearestNeighborsFunctionSerial) {
        auto birrt_nearest_neighbors_fn
                = simple_rrt_planner
                ::MakeKinematicLinearBiRRTNearestNeighborsFunction<T>(
                        EuclideanDistanceFunction, false);
        T query = (T(2) << 0.0, 0.0).finished();
        const simple_rrt_planner::SimpleRRTPlannerTree<T> &tree = getTree();
        int64_t index = birrt_nearest_neighbors_fn(
                tree,
                query,
                simple_rrt_planner::BiRRTActiveTreeType::START_TREE
        );
        GTEST_ASSERT_EQ(index, 0);
    }

    GTEST_TEST(ExtraTests, BuildPRMForPuzzlePlanningProblem) {
        GraphPuzzle puzzle{50, 50};
        Graph<T> roadmap;
        const int map_size = 1000;
        const int K = 10;
        GrowRoadMapOnPlanningProblem(
                roadmap,
                puzzle,
                map_size,
                K,
                true /* use parallel */
        );
        GTEST_ASSERT_GE(roadmap.Size(), map_size);
    }
}
