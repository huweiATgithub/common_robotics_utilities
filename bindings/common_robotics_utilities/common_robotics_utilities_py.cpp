#include <pybind11/pybind11.h>
#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/operators.h"
#include "pybind11/stl.h"
#include "helper_functions_mainly_for_python.hpp"


namespace py = pybind11;

void export_common_robotics_utilities_core(py::module_&);

PYBIND11_MODULE(common_robotics_utilities, m){
    export_common_robotics_utilities_core(m);
}


    {
        using Class = Graph<T>;
        py::class_<Class>(m, "Graph", "")
                .def(py::init<>(), "")
                .def(py::init<const size_t>(), py::arg("expected_size"), "")
                .def(py::init<const std::vector<GraphNode<T>,
                             Eigen::aligned_allocator<GraphNode<T>>> &>(),
                     py::arg("nodes"), "")
                .def("MakePrunedCopy", &Class::MakePrunedCopy,
                     py::arg("nodes_to_prune"), py::arg("use_parallel"), "")
                .def("Size", &Class::Size, "")
                .def("IndexInRange", &Class::IndexInRange, py::arg("index"), "")
                .def("CheckGraphLinkage", &Class::CheckGraphLinkage, "")
                .def("GetNodesImmutable", &Class::GetNodesImmutable, "")
                .def("GetNodesMutable", &Class::GetNodesMutable, "")
                .def("GetNodeImmutable", &Class::GetNodeImmutable, py::arg("index"), "")
                .def("AddNode", py::overload_cast<const GraphNode<T> &>(&Class::AddNode),
                     py::arg("new_node"), "")
                .def("AddNode", py::overload_cast<const T &>(&Class::AddNode),
                     py::arg("new_value"), "")
                .def("AddEdgeBetweenNodes", &Class::AddEdgeBetweenNodes,
                     py::arg("from_index"), py::arg("to_index"), py::arg("edge_weight"),
                     "")
                .def("__str__", &Class::Print)
                .def(py::pickle(
                        [](const Class &self) { return self.GetNodesImmutable(); }, // __getstate__
                        [](const std::vector<GraphNode<T>, Eigen::aligned_allocator<GraphNode<T>>> &
                        node) { return Class(node); })); // __setstate__
    }

    // PRM
    {
        using Class = NNDistanceDirection;
        py::class_<Class>(m, "NNDistanceDirection", "").def(py::init<>(), "");
    }

    m.def("AddNodeToRoadmap", &AddNodeToRoadmap<T, Graph<T>>,
          py::arg("state"),
          py::arg("nn_distance_direction"), py::arg("roadmap"),
          py::arg("distance_fn"), py::arg("edge_validity_check_fn"), py::arg("K"),
          py::arg("max_node_index_for_knn"),
          py::arg("use_parallel") = true, py::arg("connection_is_symmetric") = true,
          py::arg("add_duplicate_states") = false, "");

    m.def("GrowRoadMap", &GrowRoadMap<T, Graph<T>>, py::arg("roadmap"),
          py::arg("sampling_fn"), py::arg("distance_fn"),
          py::arg("state_validity_check_fn"), py::arg("edge_validity_check_fn"),
          py::arg("termination_check_fn"), py::arg("K"),
          py::arg("use_parallel") = true, py::arg("connection_is_symmetric") = true,
          py::arg("add_duplicate_states") = false, "");

    m.def("BuildRoadMap", &BuildRoadMap<T, Graph<T>>, py::arg("roadmap_size"),
          py::arg("sampling_fn"), py::arg("distance_fn"),
          py::arg("state_validity_check_fn"), py::arg("edge_validity_check_fn"),
          py::arg("K"), py::arg("max_valid_sample_tries"),
          py::arg("use_parallel") = true, py::arg("connection_is_symmetric") = true,
          py::arg("add_duplicate_states") = false, "");

    m.def("UpdateRoadMapEdges", &UpdateRoadMapEdges<T, Graph<T>>,
          py::arg("roadmap"), py::arg("edge_validity_check_fn"),
          py::arg("distance_fn"), py::arg("use_parallel") = true, "");

    m.def("ExtractSolution", &ExtractSolution<T, std::vector<T>, Graph<T>>,
          py::arg("roadmap"), py::arg("astar_index_solution"), "");

    m.def("LazyQueryPathAndAddNodes",
          &LazyQueryPathAndAddNodes<T, std::vector<T>, Graph<T>>, py::arg("starts"),
          py::arg("goals"), py::arg("roadmap"), py::arg("distance_fn"),
          py::arg("edge_validity_check_fn"), py::arg("K"),
          py::arg("use_parallel") = true, py::arg("connection_is_symmetric") = true,
          py::arg("add_duplicate_states") = false,
          py::arg("limit_astar_pqueue_duplicates") = true, "");

    m.def("QueryPathAndAddNodes",
          &QueryPathAndAddNodes<T, std::vector<T>, Graph<T>>, py::arg("starts"),
          py::arg("goals"), py::arg("roadmap"), py::arg("distance_fn"),
          py::arg("edge_validity_check_fn"), py::arg("K"),
          py::arg("use_parallel") = true, py::arg("connection_is_symmetric") = true,
          py::arg("add_duplicate_states") = false,
          py::arg("limit_astar_pqueue_duplicates") = true, "");

    m.def("LazyQueryPath", &LazyQueryPath<T, std::vector<T>, Graph<T>>,
          py::arg("starts"), py::arg("goals"), py::arg("roadmap"),
          py::arg("distance_fn"), py::arg("edge_validity_check_fn"), py::arg("K"),
          py::arg("use_parallel") = true, py::arg("connection_is_symmetric") = true,
          py::arg("add_duplicate_states") = false,
          py::arg("limit_astar_pqueue_duplicates") = true,
          py::arg("use_roadmap_overlay") = true, "");

    m.def("QueryPath", &QueryPath<T, std::vector<T>, Graph<T>>, py::arg("starts"),
          py::arg("goals"), py::arg("roadmap"), py::arg("distance_fn"),
          py::arg("edge_validity_check_fn"), py::arg("K"),
          py::arg("use_parallel") = true, py::arg("connection_is_symmetric") = true,
          py::arg("add_duplicate_states") = false,
          py::arg("limit_astar_pqueue_duplicates") = true,
          py::arg("use_roadmap_overlay") = true, "");

}