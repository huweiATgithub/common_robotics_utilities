import pickle
from typing import List

import numpy as np

# noinspection PyUnresolvedReferences
from common_robotics_utilities import (
    SimpleRRTPlannerState,
    SimpleRRTPlannerTree,
    PropagatedState,
    MakeKinematicLinearRRTNearestNeighborsFunction,
    MakeRRTTimeoutTerminationFunction,
    RRTPlanSinglePath,
    MakeKinematicLinearBiRRTNearestNeighborsFunction,
    MakeBiRRTTimeoutTerminationFunction,
    BiRRTPlanSinglePath,
    QueryPath,
    LazyQueryPath,
    Graph,
    GrowRoadMap,
    UpdateRoadMapEdges,
    MakeUniformRandomBiRRTSelectActiveTreeFunction,
    MakeUniformRandomBiRRTSelectSampleTypeFunction,
    MakeUniformRandomBiRRTTreeSamplingFunction,
    EuclideanDistanceFunction,
    GraphPuzzle,
    GrowRoadMapOnPlanningProblem,
)

distance_fn = EuclideanDistanceFunction


class Problem:
    def __init__(self, goal: np.ndarray):
        self.goal = goal

    def check_goal_fn(self, sample) -> bool:
        return self.is_connected(sample, self.goal)

    def goal_bias_sampling_fn(self) -> np.ndarray:
        goal_bias = 0.05
        if np.random.rand() < goal_bias:
            return self.goal
        return self.sampling_fn()

    @staticmethod
    def sampling_fn() -> np.ndarray:
        return np.random.rand(2) * 3

    @staticmethod
    def is_connected(node1, node2):
        return distance_fn(node1, node2) < 1e-6

    # noinspection PyUnusedLocal
    @classmethod
    def is_connected_active_tree(cls, node1, node2, active_tree_type):
        return cls.is_connected(node1, node2)

    @staticmethod
    def not_allow(node) -> bool:
        return 1 <= node[0] <= 2 and node[1] <= 2

    @classmethod
    def can_connect(cls, node1, node2) -> bool:
        check_step = 0.01

        check_dist = distance_fn(node1, node2)
        for ii in range(1, int(check_dist / check_step)):
            check_point = node1 + ii * check_step / check_dist * (node2 - node1)
            if cls.not_allow(check_point):
                return False
        return True

    @classmethod
    def extend_fn(cls, nearest, sample):
        step_size = 0.1

        if cls.not_allow(sample):
            return []

        extend_dist = distance_fn(nearest, sample)
        if extend_dist <= step_size:
            extend = sample
        else:
            extend = nearest + step_size / extend_dist * (sample - nearest)

        if cls.not_allow(extend) or not cls.can_connect(nearest, extend):
            return []
        return [PropagatedState(state=extend, relative_parent_index=-1)]

    # noinspection PyUnusedLocal
    @classmethod
    def extend_active_tree_fn(cls, nearest, sample, active_tree_type):
        return cls.extend_fn(nearest, sample)

    def verify_path(self, path: List[np.ndarray]):
        assert self.check_goal_fn(path[-1])
        for p in path:
            assert not self.not_allow(p), f"Node {p} is not allowed!"
        for p1, p2 in zip(path[:-1], path[1:]):
            assert self.can_connect(p1, p2), f"Nodes {p1} and {p2} can not connect!"


def test_rrt():
    np.random.seed(22)

    start = np.array([0.5, 0.5])
    goal = np.array([2.5, 0.5])
    solve_timeout = 5
    problem = Problem(goal)
    rrt_tree = SimpleRRTPlannerTree([SimpleRRTPlannerState(start)])

    nearest_neighbor_fn = MakeKinematicLinearRRTNearestNeighborsFunction(
        distance_fn=distance_fn, use_parallel=True
    )
    termination_fn = MakeRRTTimeoutTerminationFunction(solve_timeout)

    single_result = RRTPlanSinglePath(
        tree=rrt_tree,
        sampling_fn=problem.goal_bias_sampling_fn,
        nearest_neighbor_fn=nearest_neighbor_fn,
        forward_propagation_fn=problem.extend_fn,
        state_added_callback_fn=None,
        check_goal_reached_fn=problem.check_goal_fn,
        goal_reached_callback_fn=None,
        termination_check_fn=termination_fn,
    )
    problem.verify_path(single_result.Path())


def test_birrt():
    np.random.seed(23)

    start = np.array([0.5, 0.5])
    goal = np.array([2.5, 0.5])
    step_size = 0.1
    solve_timeout = 5

    problem = Problem(goal)

    def connect_fn(nearest, sample, active_tree_type):  # noqa
        if problem.not_allow(sample):
            return []

        total_dist = distance_fn(nearest, sample)
        total_steps = int(np.ceil(total_dist / step_size))

        propagated_states = []
        parent_offset = -1
        current = nearest
        for steps in range(total_steps):
            target_dist = distance_fn(current, sample)
            if target_dist > step_size:
                current_target = current + step_size / target_dist * (sample - current)
            elif target_dist < 1e-6:
                break
            else:
                current_target = sample

            if problem.not_allow(current_target) or not problem.can_connect(
                current, current_target
            ):
                return propagated_states
            propagated_states.append(
                PropagatedState(
                    state=current_target, relative_parent_index=parent_offset
                )
            )
            parent_offset += 1
            current = current_target

        return propagated_states

    nearest_neighbor_fn = MakeKinematicLinearBiRRTNearestNeighborsFunction(
        distance_fn=distance_fn, use_parallel=True
    )
    termination_fn = MakeBiRRTTimeoutTerminationFunction(solve_timeout)
    select_sample_type_fn = MakeUniformRandomBiRRTSelectSampleTypeFunction(
        np.random.rand, 0.5
    )
    tree_sampling_fn = MakeUniformRandomBiRRTTreeSamplingFunction(np.random.rand)
    select_active_tree_fn = MakeUniformRandomBiRRTSelectActiveTreeFunction(
        np.random.rand, 0.25
    )

    args_dict = {
        "select_sample_type_fn": select_sample_type_fn,
        "state_sampling_fn": problem.sampling_fn,
        "tree_sampling_fn": tree_sampling_fn,
        "nearest_neighbor_fn": nearest_neighbor_fn,
        "state_added_callback_fn": None,
        "states_connected_fn": problem.is_connected_active_tree,
        "goal_bridge_callback_fn": None,
        "select_active_tree_fn": select_active_tree_fn,
        "termination_check_fn": termination_fn,
    }

    extend_result = BiRRTPlanSinglePath(
        start_tree=SimpleRRTPlannerTree([SimpleRRTPlannerState(start)]),
        goal_tree=SimpleRRTPlannerTree([SimpleRRTPlannerState(goal)]),
        propagation_fn=problem.extend_active_tree_fn,
        **args_dict,
    )
    problem.verify_path(extend_result.Path())

    connect_result = BiRRTPlanSinglePath(
        start_tree=SimpleRRTPlannerTree([SimpleRRTPlannerState(start)]),
        goal_tree=SimpleRRTPlannerTree([SimpleRRTPlannerState(goal)]),
        propagation_fn=connect_fn,
        **args_dict,
    )
    problem.verify_path(connect_result.Path())


def test_prm():
    np.random.seed(42)
    # An array of string
    test_env = np.array(
        [
            "####################",
            "#                  #",
            "#  ####            #",
            "#  ####    #####   #",
            "#  ####    #####   #",
            "#          #####   #",
            "#          #####   #",
            "#                  #",
            "#      #########   #",
            "#     ##########   #",
            "#    ###########   #",
            "#   ############   #",
            "#                  #",
            "#                  #",
            "#    ##            #",
            "#    ##   ######## #",
            "#    ##   ######## #",
            "#    ##   ######## #",
            "#                  #",
            "####################",
        ]
    )

    test_env_shape = [len(test_env[0]), len(test_env)]

    K = 5
    roadmap_size = 100

    def roadmap_termination_fn(current_roadmap_size):
        return current_roadmap_size >= roadmap_size

    def state_sampling_fn():
        x = np.random.randint(test_env_shape[0])
        y = np.random.randint(test_env_shape[1])

        return np.array([x, y])

    def check_state_validity_fn(point):
        x, y = point
        return test_env[int(y)][int(x)] != "#"

    def check_edge_collision_free(start, end, step_size):
        num_steps = np.ceil(distance_fn(start, end) / step_size)

        for step in range(int(num_steps) + 1):
            interpolation_ratio = step / num_steps
            interpolated_point = start + np.round(interpolation_ratio * (end - start))

            if not check_state_validity_fn(interpolated_point):
                return False
        return True

    def check_edge_validity_fn(start, end):
        return check_edge_collision_free(start, end, 0.5) and check_edge_collision_free(
            end, start, 0.5
        )

    # for plan checking
    def set_cell(env, point, char):
        x, y = point
        x, y = int(x), int(y)
        env[y] = env[y][:x] + char + env[y][x + 1 :]

    def get_cell(env, point):  # noqa
        x, y = point
        return env[int(y)][int(x)]

    def draw_environment(env):
        print("".join(list(map(lambda row: row + "\n", env))))

    def draw_path(env, starts_, goals_, path_):
        tmp_env = env.copy()
        for p in path_:
            set_cell(tmp_env, p, "+")
        for start in starts_:
            set_cell(tmp_env, start, "S")
        for goal in goals_:
            set_cell(tmp_env, goal, "G")

        draw_environment(tmp_env)

    def check_path(path_):
        assert len(path_) >= 2

        for idx in range(1, len(path_)):
            # We check both forward and backward because rounding in the
            # waypoint interpolation can create edges that are valid in
            # only one direction.
            forward_valid = check_edge_validity_fn(path_[idx - 1], path_[idx])
            backward_valid = check_edge_validity_fn(path_[idx], path_[idx - 1])

            edge_valid = forward_valid and backward_valid

            assert edge_valid

    def check_plan(starts_, goals_, path_):
        draw_path(test_env, starts_, goals_, path_)

        print("Checking raw path")
        check_path(path_)

    roadmap = Graph()

    GrowRoadMap(
        roadmap,
        state_sampling_fn,
        distance_fn,
        check_state_validity_fn,
        check_edge_validity_fn,
        roadmap_termination_fn,
        K,
        use_parallel=False,
        connection_is_symmetric=True,
        add_duplicate_states=False,
    )
    assert roadmap.CheckGraphLinkage()

    UpdateRoadMapEdges(roadmap, check_edge_validity_fn, distance_fn, False)
    assert roadmap.CheckGraphLinkage()

    nodes_to_prune = {10, 20, 30, 40, 50, 60}
    serial_pruned_roadmap = roadmap.MakePrunedCopy(nodes_to_prune, False)
    assert serial_pruned_roadmap.CheckGraphLinkage()

    parallel_pruned_roadmap = roadmap.MakePrunedCopy(nodes_to_prune, True)
    assert parallel_pruned_roadmap.CheckGraphLinkage()

    # test planning
    keypoints = [
        np.array([1, 1]),
        np.array([18, 18]),
        np.array([7, 13]),
        np.array([9, 5]),
    ]

    for _start in keypoints:
        for _goal in keypoints:
            if np.array_equal(_start, _goal):
                continue

            print(f"PRM Path ({_start} to {_goal})")
            path = QueryPath(
                [_start],
                [_goal],
                roadmap,
                distance_fn,
                check_edge_validity_fn,
                K,
                use_parallel=False,
                connection_is_symmetric=True,
                add_duplicate_states=False,
                limit_astar_pqueue_duplicates=True,
            ).Path()
            check_plan([_start], [_goal], path)

            print(f"Lazy-PRM Path ({_start} to {_goal})")

            lazy_path = LazyQueryPath(
                [_start],
                [_goal],
                roadmap,
                distance_fn,
                check_edge_validity_fn,
                K,
                use_parallel=False,
                connection_is_symmetric=True,
                add_duplicate_states=False,
                limit_astar_pqueue_duplicates=True,
            ).Path()

            check_plan([_start], [_goal], lazy_path)

    starts = [keypoints[0], keypoints[1]]
    goals = [keypoints[2], keypoints[3]]
    print(f"Multi start/goal PRM Path ({starts} to {goals})")
    multi_path = QueryPath(
        starts,
        goals,
        roadmap,
        distance_fn,
        check_edge_validity_fn,
        K,
        use_parallel=False,
        connection_is_symmetric=True,
        add_duplicate_states=False,
        limit_astar_pqueue_duplicates=True,
    ).Path()

    check_plan(starts, goals, multi_path)

    graph_dump = pickle.dumps(roadmap)
    graph_recovered = pickle.loads(graph_dump)
    assert str(roadmap) == str(graph_recovered)

    puzzle = GraphPuzzle(len(test_env), len(test_env[0]))
    roadmap = Graph()
    pos_obstacles = []
    for i in range(len(test_env)):
        for j in range(len(test_env[0])):
            if test_env[i][j] == "#":
                pos_obstacles.append(np.array([i, j]))
    puzzle.set(pos_obstacles)

    assert roadmap.Size() == 0
    GrowRoadMapOnPlanningProblem(
        roadmap,
        puzzle,
        roadmap_size,
        K,
        use_parallel=True,
        connection_is_symmetric=True,
        add_duplicate_states=False,
    )
    assert roadmap.Size() >= roadmap_size
