/*
 * This unit defines some extra functions we added to common-robotics-utilities
 */
#include <pybind11/pybind11.h>
#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/operators.h"
#include "pybind11/stl.h"
#include "helper_functions_mainly_for_python.hpp"


namespace py = pybind11;
void export_common_robotics_utilities_extra(py::module_& m) {
    using namespace common_robotics_utilities::extras;

    m.def("EuclideanDistanceFunction", EuclideanDistanceFunction);
    using T = Eigen::VectorXd;
    {
        py::class_<PlanningProblem<T>>(m, "PlanningProblem", ""); // NOLINT(bugprone-unused-raii)
        using Class = GraphPuzzle;
        py::class_<Class, PlanningProblem<T>>(m, "GraphPuzzle", "")
                .def(py::init<int, int>(),
                     py::arg("rows"), py::arg("cols"),
                     "")
                .def("set", &Class::set, "")
                .def("state_sampling_fn", &Class::state_sampling_fn, "")
                .def("check_state_validity_fn", &Class::check_state_validity_fn, "")
                .def("check_edge_validity_fn", &Class::check_edge_validity_fn, "")
                .def("get_distance_fn", &Class::get_std_distance_fn, "");

        m.def("GrowRoadMapOnPlanningProblem", &GrowRoadMapOnPlanningProblem<T>,
              py::arg("roadmap"), py::arg("problem"),
              py::arg("map_size"), py::arg("K"),
              py::arg("use_parallel") = true, py::arg("connection_is_symmetric") = true,
              py::arg("add_duplicate_states") = false, "");

    }

}