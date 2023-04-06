include(FetchContent)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11
    GIT_TAG        v2.10.4
)
FetchContent_MakeAvailable(pybind11)

set(PY_BINDING_NAME "${PROJECT_NAME}_py")
include_directories(common_robotics_utilities)
pybind11_add_module(
    ${PY_BINDING_NAME}
    common_robotics_utilities/common_robotics_utilities_py.cpp
    common_robotics_utilities/common_robotics_utilities_original_py.cpp
    common_robotics_utilities/common_robotics_utilities_extra_py.cpp
)
# Change the name of output library to match the one defined in `..._py.cpp` and the one used in `__init__.py`
# https://github.com/pybind/pybind11/issues/415
set_target_properties(${PY_BINDING_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
add_dependencies(${PY_BINDING_NAME} ${PROJECT_NAME})
target_link_libraries(${PY_BINDING_NAME} PRIVATE ${PROJECT_NAME})

if (NOT SKBUILD)
    add_executable(extra_tests test/extra_tests.cpp)
    target_link_libraries(extra_tests ${PROJECT_NAME} GTest::gtest_main)
    gtest_discover_tests(extra_tests)
endif ()

# Installation
if (NOT SKBUILD)
    install(DIRECTORY ${PROJECT_NAME} DESTINATION . FILES_MATCHING PATTERN "*.py")
endif ()
install(TARGETS ${PY_BINDING_NAME} DESTINATION ${PROJECT_NAME} COMPONENT library)
install(TARGETS ${PROJECT_NAME} DESTINATION ${PROJECT_NAME} COMPONENT library)
