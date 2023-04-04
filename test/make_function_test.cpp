#include <gtest/gtest.h>
#include <common_robotics_utilities/simple_rrt_planner.hpp>
#include <common_robotics_utilities/helper_functions_mainly_for_python.hpp>
#include <Eigen/Core>

namespace common_robotics_utilities::make_function_test {
        using T = Eigen::VectorXd;

        static simple_rrt_planner::SimpleRRTPlannerTree<T>& getTree(){
            int num_nodes = 100;
            static simple_rrt_planner::SimpleRRTPlannerTree<T> tree(num_nodes);
            for(int i=0;i<num_nodes;i++){
                tree.AddNode((T(2) << i / 10.0, i / 10.0).finished());
            }
            return tree;
        }

        GTEST_TEST(MakeFunctionTest, MakeKinematicLinearBiRRTNearestNeighborsFunctionParallel) {
            auto birrt_nearest_neighbors_fn
                    = simple_rrt_planner
                    ::MakeKinematicLinearBiRRTNearestNeighborsFunction<T>(
                            EuclideanDistanceFunction, true);
            T query = (T(2) << 0.0, 0.0).finished();
            const simple_rrt_planner::SimpleRRTPlannerTree<T>& tree = getTree();
            int64_t index = birrt_nearest_neighbors_fn(
                    tree,
                    query,
                    simple_rrt_planner::BiRRTActiveTreeType::START_TREE
            );
            GTEST_ASSERT_EQ(index, 0);
        }
        GTEST_TEST(MakeFunctionTest, MakeKinematicLinearBiRRTNearestNeighborsFunctionSerial) {
            auto birrt_nearest_neighbors_fn
                    = simple_rrt_planner
                    ::MakeKinematicLinearBiRRTNearestNeighborsFunction<T>(
                            EuclideanDistanceFunction, false);
            T query = (T(2) << 0.0, 0.0).finished();
            const simple_rrt_planner::SimpleRRTPlannerTree<T>& tree = getTree();
            int64_t index = birrt_nearest_neighbors_fn(
                    tree,
                    query,
                    simple_rrt_planner::BiRRTActiveTreeType::START_TREE
            );
            GTEST_ASSERT_EQ(index, 0);
        }
    }

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
