import numpy as np

from common_robotics_utilities import (
    MakeKinematicLinearBiRRTNearestNeighborsFunction,
    SimpleRRTPlannerTree,
    SimpleRRTPlannerState,
    BiRRTActiveTreeType,
    EuclideanDistanceFunction,
)


def distance_fn(q1, q2):
    return np.linalg.norm(q1 - q2)


def test_kinematic_linear_birrt_nearest_neighbors_function():
    nodes = np.linspace(np.array([0.0, 0.0]), np.array([1.0, 1.0]), 100)
    query = np.array([0.0, 0.0])
    start_tree = SimpleRRTPlannerTree([SimpleRRTPlannerState(node) for node in nodes])

    nearest_neighbor_fn = MakeKinematicLinearBiRRTNearestNeighborsFunction(
        distance_fn=distance_fn,
        use_parallel=False,
    )
    nearest_index = nearest_neighbor_fn(
        start_tree, query, BiRRTActiveTreeType.START_TREE
    )
    assert nearest_index == 0

    # Use distance fn defined in Python with parallel (OMP_FOR) will hang the program
    nearest_neighbor_parallel_fn = MakeKinematicLinearBiRRTNearestNeighborsFunction(
        EuclideanDistanceFunction,
        use_parallel=True,
    )
    nearest_index = nearest_neighbor_parallel_fn(
        start_tree, query, BiRRTActiveTreeType.START_TREE
    )
    assert nearest_index == 0
