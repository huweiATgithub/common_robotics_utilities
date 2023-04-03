import pickle
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
)


def test_rrt():
    start = np.array([0.5, 0.5])
    goal = np.array([2.5, 0.5])
    goal_bias = 0.05
    step_size = 0.1
    check_step = 0.01
    solve_timeout = 2

    rrt_tree = SimpleRRTPlannerTree([SimpleRRTPlannerState(start)])

    def sampling_fn():
        if np.random.rand() < goal_bias:
            return goal
        return np.random.rand(2) * 3

    def distance_fn(point1, point2):
        return np.linalg.norm(point2 - point1)

    def check_goal_fn(sample):
        return np.linalg.norm(sample - goal) < 1e-6

    def extend_fn(nearest, sample):
        if 1 <= sample[0] <= 2 and sample[1] <= 2:
            return []

        extend_dist = distance_fn(nearest, sample)
        if extend_dist <= step_size:
            extend = sample
        else:
            extend = nearest + step_size / extend_dist * (sample - nearest)

        check_dist = distance_fn(nearest, extend)
        for ii in range(1, int(check_dist / check_step)):
            check_point = nearest + ii * check_step / check_dist * (extend - nearest)
            if 1 <= check_point[0] <= 2 and check_point[1] <= 2:
                return []
        return [PropagatedState(state=extend, relative_parent_index=-1)]

    nearest_neighbor_fn = MakeKinematicLinearRRTNearestNeighborsFunction(
        distance_fn=distance_fn, use_parallel=False
    )
    termination_fn = MakeRRTTimeoutTerminationFunction(solve_timeout)

    single_result = RRTPlanSinglePath(
        tree=rrt_tree,
        sampling_fn=sampling_fn,
        nearest_neighbor_fn=nearest_neighbor_fn,
        forward_propagation_fn=extend_fn,
        state_added_callback_fn=None,
        check_goal_reached_fn=check_goal_fn,
        goal_reached_callback_fn=None,
        termination_check_fn=termination_fn,
    )

    print(single_result.Path())


def test_birrt():
    start = np.array([0.5, 0.5])
    goal = np.array([2.5, 0.5])
    step_size = 0.1
    check_step = 0.01
    solve_timeout = 2

    start_tree = SimpleRRTPlannerTree([SimpleRRTPlannerState(start)])
    goal_tree = SimpleRRTPlannerTree([SimpleRRTPlannerState(goal)])

    def sampling_fn():
        return np.random.rand(2) * 3

    def distance_fn(point1, point2):
        return np.linalg.norm(point2 - point1)

    def extend_fn(nearest, sample, is_start_tree):  # noqa
        if 1 <= sample[0] <= 2 and sample[1] <= 2:
            return []

        extend_dist = distance_fn(nearest, sample)
        if extend_dist <= step_size:
            extend = sample
        else:
            extend = nearest + step_size / extend_dist * (sample - nearest)

        check_dist = distance_fn(nearest, extend)
        for ii in range(1, int(check_dist / check_step)):
            check_point = nearest + ii * check_step / check_dist * (extend - nearest)
            if 1 <= check_point[0] <= 2 and check_point[1] <= 2:
                return []
        return [PropagatedState(state=extend, relative_parent_index=-1)]

    def connect_fn(nearest, sample, is_start_tree):  # noqa
        if 1 <= sample[0] <= 2 and sample[1] <= 2:
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

            check_dist = distance_fn(current, current_target)
            for ii in range(1, int(check_dist / check_step)):
                check_point = current + ii * check_step / check_dist * (
                    current_target - current
                )
                if 1 <= check_point[0] <= 2 and check_point[1] <= 2:
                    return propagated_states
            propagated_states.append(
                PropagatedState(
                    state=current_target, relative_parent_index=parent_offset
                )
            )
            parent_offset += 1
            current = current_target

        return propagated_states

    # noinspection PyUnusedLocal
    def states_connected_fn(source, target, is_start_tree):
        return np.linalg.norm(source - target) < 1e-6

    nearest_neighbor_fn = MakeKinematicLinearBiRRTNearestNeighborsFunction(
        distance_fn=distance_fn, use_parallel=False
    )

    termination_fn = MakeBiRRTTimeoutTerminationFunction(solve_timeout)

    select_sample_type_fn = MakeUniformRandomBiRRTSelectSampleTypeFunction(
        np.random.rand, 0.5
    )
    tree_sampling_fn = MakeUniformRandomBiRRTTreeSamplingFunction(np.random.rand)
    select_active_tree_fn = MakeUniformRandomBiRRTSelectActiveTreeFunction(
        np.random.rand, 0.25
    )
    BiRRTPlanSinglePath(
        start_tree=start_tree,
        goal_tree=goal_tree,
        select_sample_type_fn=select_sample_type_fn,
        state_sampling_fn=sampling_fn,
        tree_sampling_fn=tree_sampling_fn,
        nearest_neighbor_fn=nearest_neighbor_fn,
        propagation_fn=extend_fn,
        state_added_callback_fn=None,
        states_connected_fn=states_connected_fn,
        goal_bridge_callback_fn=None,
        select_active_tree_fn=select_active_tree_fn,
        termination_check_fn=termination_fn,
    )

    BiRRTPlanSinglePath(
        start_tree=start_tree,
        goal_tree=goal_tree,
        select_sample_type_fn=MakeUniformRandomBiRRTSelectSampleTypeFunction(
            np.random.rand, 0.5
        ),
        state_sampling_fn=sampling_fn,
        tree_sampling_fn=MakeUniformRandomBiRRTTreeSamplingFunction(np.random.rand),
        nearest_neighbor_fn=nearest_neighbor_fn,
        propagation_fn=connect_fn,
        state_added_callback_fn=None,
        states_connected_fn=states_connected_fn,
        goal_bridge_callback_fn=None,
        select_active_tree_fn=MakeUniformRandomBiRRTSelectActiveTreeFunction(
            np.random.rand, 0.25
        ),
        termination_check_fn=termination_fn,
    )


def test_prm():
    np.random.seed(42)
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

    def distance_fn(start, end):
        return np.linalg.norm(end - start)

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
        False,
        True,
        False,
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
