cmake_minimum_required(VERSION 3.15)
project(common_robotics_utilities)

find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(OpenMP)

include_directories(include SYSTEM ${Eigen3_INCLUDE_DIRS})

set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Werror)
add_compile_options(-Wconversion)
add_compile_options(-Wshadow)
add_compile_options(-O3)
add_compile_options(-g)
add_compile_options(-Werror=non-virtual-dtor)
add_compile_options(-Wold-style-cast)
add_compile_options(-Wpessimizing-move)
add_compile_options(-Wuninitialized)

## It's not clear if add_compile_options does the right things for flags that
## may differ between languages and target type.
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS
    "${CMAKE_SHARED_LINKER_FLAGS} ${OpenMP_SHARED_LINKER_FLAGS}")

# Utility library
add_library(${PROJECT_NAME}
            include/${PROJECT_NAME}/base64_helpers.hpp
            include/${PROJECT_NAME}/color_builder.hpp
            include/${PROJECT_NAME}/conversions.hpp
            include/${PROJECT_NAME}/dynamic_spatial_hashed_voxel_grid.hpp
            include/${PROJECT_NAME}/gaussian_distributions.hpp
            include/${PROJECT_NAME}/math.hpp
            include/${PROJECT_NAME}/maybe.hpp
            include/${PROJECT_NAME}/openmp_helpers.hpp
            include/${PROJECT_NAME}/path_processing.hpp
            include/${PROJECT_NAME}/print.hpp
            include/${PROJECT_NAME}/random_rotation_generator.hpp
            include/${PROJECT_NAME}/serialization.hpp
            include/${PROJECT_NAME}/simple_astar_search.hpp
            include/${PROJECT_NAME}/simple_dtw.hpp
            include/${PROJECT_NAME}/simple_graph.hpp
            include/${PROJECT_NAME}/simple_graph_search.hpp
            include/${PROJECT_NAME}/simple_hausdorff_distance.hpp
            include/${PROJECT_NAME}/simple_hierarchical_clustering.hpp
            include/${PROJECT_NAME}/simple_kmeans_clustering.hpp
            include/${PROJECT_NAME}/simple_knearest_neighbors.hpp
            include/${PROJECT_NAME}/simple_prm_planner.hpp
            include/${PROJECT_NAME}/simple_prngs.hpp
            include/${PROJECT_NAME}/simple_robot_model_interface.hpp
            include/${PROJECT_NAME}/simple_rrt_planner.hpp
            include/${PROJECT_NAME}/simple_task_planner.hpp
            include/${PROJECT_NAME}/time_optimal_trajectory_parametrization.hpp
            include/${PROJECT_NAME}/utility.hpp
            include/${PROJECT_NAME}/voxel_grid.hpp
            include/${PROJECT_NAME}/zlib_helpers.hpp
            src/${PROJECT_NAME}/base64_helpers.cpp
            src/${PROJECT_NAME}/conversions.cpp
            src/${PROJECT_NAME}/math.cpp
            src/${PROJECT_NAME}/serialization.cpp
            src/${PROJECT_NAME}/time_optimal_trajectory_parametrization.cpp
            src/${PROJECT_NAME}/zlib_helpers.cpp)
target_link_libraries(${PROJECT_NAME} z)

# Examples
if (NOT SKBUILD)
    add_executable(clustering_example example/clustering_example.cpp)
    add_dependencies(clustering_example ${PROJECT_NAME})
    target_link_libraries(clustering_example ${PROJECT_NAME})

    add_executable(dtw_example example/dtw_example.cpp)
    add_dependencies(dtw_example ${PROJECT_NAME})
    target_link_libraries(dtw_example ${PROJECT_NAME})

# Tests
    include(FetchContent)
    FetchContent_Declare(
      googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG b5fd99bbd55ebe1a3488b8ea3717fba089293457
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF)  # don't install tests
    FetchContent_MakeAvailable(googletest)

    enable_testing()
    include(GoogleTest)

    add_executable(maybe_test test/maybe_test.cpp)
    target_link_libraries(maybe_test ${PROJECT_NAME} GTest::gtest)
    gtest_discover_tests(maybe_test)

    add_executable(planning_test test/planning_test.cpp)
    target_link_libraries(planning_test ${PROJECT_NAME} GTest::gtest)
    gtest_discover_tests(planning_test)

    add_executable(task_planning_test test/task_planning_test.cpp)
    target_link_libraries(task_planning_test ${PROJECT_NAME} GTest::gtest)
    gtest_discover_tests(task_planning_test)

    add_executable(utility_test test/utility_test.cpp)
    target_link_libraries(utility_test ${PROJECT_NAME} GTest::gtest)
    gtest_discover_tests(utility_test)

    add_executable(voxel_grid_test test/voxel_grid_test.cpp)
    target_link_libraries(voxel_grid_test ${PROJECT_NAME} GTest::gtest)
    gtest_discover_tests(voxel_grid_test)
endif ()


# Bindings
add_subdirectory(bindings)
