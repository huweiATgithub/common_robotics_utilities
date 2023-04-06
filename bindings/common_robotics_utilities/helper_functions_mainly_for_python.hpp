#pragma once

#include <common_robotics_utilities/openmp_helpers.hpp>
#include <common_robotics_utilities/simple_prm_planner.hpp>
#include <Eigen/Core>
#include <cmath>
#include <functional>
#include <random>

namespace common_robotics_utilities::extras {
    inline double EuclideanDistanceFunction(const Eigen::VectorXd &v1,
                                            const Eigen::VectorXd &v2) {
        return (v1 - v2).squaredNorm();
    }

    template<typename StateType>
    class PlanningProblem {
    public:
        typedef double (*raw_distance_function_type)(const StateType &, const StateType &);

        typedef std::function<double(const StateType &, const StateType &)> std_distance_function_type;

        virtual ~PlanningProblem() = default;

        [[nodiscard]] virtual StateType state_sampling_fn() const = 0;

        [[nodiscard]] virtual bool check_state_validity_fn(const StateType &point) const = 0;

        [[nodiscard]] virtual bool check_edge_validity_fn(const StateType &start, const StateType &end) const = 0;

        [[nodiscard]] virtual raw_distance_function_type get_distance_fn() const = 0;

        [[nodiscard]] virtual std_distance_function_type get_std_distance_fn() const {
            // std::function is needed for pybind11
            // std::function is polymorphic wrapper which cannot be made constexpr
            // (cannot make it compile time).
            return get_distance_fn();
        }
    };

    class GraphPuzzle : public PlanningProblem<Eigen::VectorXd> {
    public:
        GraphPuzzle(int rows, int cols) : n_rows_(rows), n_cols_(cols) {
            map_ = new bool[static_cast<unsigned int>(rows * cols)];
            clear();
            std::random_device rd;
            eng_ptr_ = std::make_unique<std::mt19937>(rd());
            row_index_dist_ =
                    std::make_unique<std::uniform_int_distribution<>>(0, rows);
            col_index_dist_ =
                    std::make_unique<std::uniform_int_distribution<>>(0, cols);
        }

        void clear() {
            memset(map_, true, static_cast<unsigned int>(n_rows_ * n_cols_));
        }

        void set(const std::vector<Eigen::VectorXd> &obs_pos) {
            clear();
            for (auto &pos: obs_pos) {
                map_[int(pos[0]) * n_cols_ + int(pos[1])] = false;
            }
        }

        ~GraphPuzzle() override { delete[] map_; }


        [[nodiscard]] Eigen::VectorXd state_sampling_fn() const override {
            Eigen::VectorXd pos(2);
            pos[0] = static_cast<double>((*row_index_dist_)(*eng_ptr_));
            pos[1] = static_cast<double>((*col_index_dist_)(*eng_ptr_));
            return pos;
        }

        [[nodiscard]] bool check_state_validity_fn(const Eigen::VectorXd &point) const override {
            return map_[int(point[0]) * n_cols_ + int(point[1])];
        }

        [[nodiscard]] bool check_edge_collision_free(const Eigen::VectorXd &start,
                                                     const Eigen::VectorXd &end, float step_size) const {
            const double distance = (get_distance_fn())(start, end);
            const int n_steps = int(ceil(distance / step_size));
            for (int i = 0; i < n_steps; i++) {
                Eigen::VectorXd interpolation = start + (end - start) * i / n_steps;
                if (!check_state_validity_fn(interpolation)) {
                    return false;
                }
            }
            return true;
        }

        bool check_edge_validity_fn(const Eigen::VectorXd &start,
                                    const Eigen::VectorXd &end) const override {
            return check_edge_collision_free(start, end, 0.5) &&
                   check_edge_collision_free(end, start, 0.5);
        }

        [[nodiscard]] raw_distance_function_type get_distance_fn() const override {
            return EuclideanDistanceFunction;
        }

    private:
        bool *map_;
        int n_rows_;
        int n_cols_;
        std::unique_ptr<std::uniform_int_distribution<>> row_index_dist_;
        std::unique_ptr<std::uniform_int_distribution<>> col_index_dist_;
        std::unique_ptr<std::mt19937> eng_ptr_;
    };

    using simple_graph::Graph;

    template<typename StateType>
    inline void GrowRoadMapOnPlanningProblem(Graph<StateType> &roadmap,
                                             const PlanningProblem<StateType> &problem,
                                             int map_size, int64_t K,
                                             const bool use_parallel = true,
                                             const bool connection_is_symmetric = true,
                                             const bool add_duplicate_states = false) {
        simple_prm_planner::GrowRoadMap<StateType, Graph<StateType>>(
                roadmap, [&]() { return problem.state_sampling_fn(); },
                    problem.get_distance_fn(),
                [&](const StateType &state) {
                    return problem.check_state_validity_fn(state);
                },
                [&](const StateType &state1, const StateType &state2) {
                    return problem.check_edge_validity_fn(state1, state2);
                },
                [map_size](const int64_t size) { return size >= map_size; }, K,
                use_parallel, connection_is_symmetric, add_duplicate_states);
    }

} // namespace common_robotics_utilities::extras
