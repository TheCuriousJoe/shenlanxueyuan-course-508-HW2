#include "essential_matrix.h"

#include <complex>

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

void CenterAndNormalizeImagePoints(const std::vector<Eigen::Vector2d>& points,
                                   std::vector<Eigen::Vector2d>* normed_points,
                                   Eigen::Matrix3d* matrix) {
    // Calculate centroid
    Eigen::Vector2d centroid(0, 0);
    for (const Eigen::Vector2d& point : points) {
        centroid += point;
    }
    centroid /= points.size();

    // Root mean square error to centroid of all points
    double rms_mean_dist = 0;
    for (const Eigen::Vector2d& point : points) {
        rms_mean_dist += (point - centroid).squaredNorm();
    }
    rms_mean_dist = std::sqrt(rms_mean_dist / points.size());

    // Compose normalization matrix
    const double norm_factor = std::sqrt(2.0) / rms_mean_dist;
    *matrix << norm_factor, 0, -norm_factor * centroid(0), 0, norm_factor,
        -norm_factor * centroid(1), 0, 0, 1;

    // Apply normalization matrix
    normed_points->resize(points.size());

    const double M_00 = (*matrix)(0, 0);
    const double M_01 = (*matrix)(0, 1);
    const double M_02 = (*matrix)(0, 2);
    const double M_10 = (*matrix)(1, 0);
    const double M_11 = (*matrix)(1, 1);
    const double M_12 = (*matrix)(1, 2);
    const double M_20 = (*matrix)(2, 0);
    const double M_21 = (*matrix)(2, 1);
    const double M_22 = (*matrix)(2, 2);

    for (size_t i = 0; i < points.size(); ++i) {
        const double p_0 = points[i](0);
        const double p_1 = points[i](1);

        const double np_0 = M_00 * p_0 + M_01 * p_1 + M_02;
        const double np_1 = M_10 * p_0 + M_11 * p_1 + M_12;
        const double np_2 = M_20 * p_0 + M_21 * p_1 + M_22;

        const double inv_np_2 = 1.0 / np_2;
        (*normed_points)[i](0) = np_0 * inv_np_2;
        (*normed_points)[i](1) = np_1 * inv_np_2;
    }
}

Eigen::Matrix3d EssentialMatrixEightPointEstimate(const std::vector<Eigen::Vector2d>& points1,
                                                  const std::vector<Eigen::Vector2d>& points2) {

    // Center and normalize image points for better numerical stability.
    std::vector<Eigen::Vector2d> normed_points1;
    std::vector<Eigen::Vector2d> normed_points2;
    Eigen::Matrix3d points1_norm_matrix;
    Eigen::Matrix3d points2_norm_matrix;
    CenterAndNormalizeImagePoints(points1, &normed_points1, &points1_norm_matrix);
    CenterAndNormalizeImagePoints(points2, &normed_points2, &points2_norm_matrix);

    Eigen::Matrix3d E;
    E.setIdentity();

    // homework4

    
    return E;
}

Eigen::Matrix3d EssentialMatrixFromPose(const Eigen::Matrix3d& R,
                                        const Eigen::Vector3d& t) {
    Eigen::Matrix3d E;
    E.setIdentity();

    // homework3
    
    
    return E;
}

