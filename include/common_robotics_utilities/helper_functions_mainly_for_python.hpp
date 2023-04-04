#pragma once

#include <Eigen/Core>

double EuclideanDistanceFunction(const Eigen::VectorXd &v1, const Eigen::VectorXd &v2) {
    return (v1 - v2).squaredNorm();
}
