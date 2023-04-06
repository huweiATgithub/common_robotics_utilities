#include <pybind11/pybind11.h>
#include "pybind11/eigen.h"
#include "pybind11/functional.h"
#include "pybind11/operators.h"
#include "pybind11/stl.h"
#include "helper_functions_mainly_for_python.hpp"


namespace py = pybind11;

void export_common_robotics_utilities_core(py::module_&);
void export_common_robotics_utilities_extra(py::module_&);

PYBIND11_MODULE(common_robotics_utilities, m){
    export_common_robotics_utilities_core(m);
    export_common_robotics_utilities_extra(m);
}


