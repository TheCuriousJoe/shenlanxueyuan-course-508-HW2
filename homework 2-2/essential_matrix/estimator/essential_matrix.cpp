#include "essential_matrix.h"

#include <complex>

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <iostream>

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
        rms_mean_dist += (point - centroid).norm();
//        rms_mean_dist += (point - centroid).squaredNorm();
    }
    rms_mean_dist = rms_mean_dist / points.size();
//    rms_mean_dist = std::sqrt(rms_mean_dist / points.size());

    // Compose normalization matrix
    const double norm_factor = std::sqrt(2) / rms_mean_dist;
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
    double newMeanDist = 0;
    double newXMean = 0;
    double newYMean = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        newMeanDist += ((*normed_points)[i]).norm();
        newXMean += abs((*normed_points)[i](0));
        newYMean += abs((*normed_points)[i](1));
    }
    std::cout << "newMeanDist: " << newMeanDist / points.size() << std::endl;
    std::cout << "newXMean: " << newXMean / points.size() << std::endl;
    std::cout << "newYMean: " << newYMean / points.size() << std::endl;
}

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
Eigen::Matrix3d EssentialMatrixEightPointEstimate(const std::vector<Eigen::Vector2d>& points1,
                                                  const std::vector<Eigen::Vector2d>& points2) {
    // Center and normalize image points for better numerical stability.
    std::vector<Eigen::Vector2d> normed_points1;
    std::vector<Eigen::Vector2d> normed_points2;
    Eigen::Matrix3d points1_norm_matrix;
    Eigen::Matrix3d points2_norm_matrix;
    CenterAndNormalizeImagePoints(points1, &normed_points1, &points1_norm_matrix);
    CenterAndNormalizeImagePoints(points2, &normed_points2, &points2_norm_matrix);

    // For more convenient visualization, scale every pixel position by 100 and move them to the image center.
    cv::Mat draw = cv::Mat::zeros(cv::Size(800, 800), CV_8UC3);
    cv::circle(draw, cv::Point(400, 400), sqrt(2) * 100, cv::Scalar(0, 0, 255), 2);
    for (int i = 0; i < points1.size(); i++){
        cv::circle(draw, cv::Point(normed_points1[i](0) * 100 + 400, normed_points1[i](1) * 100 + 400), 5, cv::Scalar(0, 255, 0), -1);
    }
    for (int i = 0; i < points2.size(); i++){
        cv::circle(draw, cv::Point(normed_points2[i](0) * 100 + 400, normed_points2[i](1) * 100 + 400), 5, cv::Scalar(255, 0, 0), -1);
    }
    cv::imshow("Visualize normalized points", draw);
    cv::waitKey(0);

    Eigen::Matrix3d E;
    E.setIdentity();

    // homework4
    Eigen::MatrixXd A(Eigen::MatrixXd::Zero(normed_points1.size(), 9));
    for (int i = 0; i < normed_points1.size(); i++){
        double u1 = normed_points1[i](0), v1 = normed_points1[i](1),
                u2 = normed_points2[i](0), v2 = normed_points2[i](1);
        A.row(i) << u1 * u2, u1 * v2, u1, v1 * u2, v1 * v2, v1, u2, v2, 1;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::MatrixXd e_hat = V.col(V.cols() - 1);
    Eigen::MatrixXd E_hat = Eigen::Map<Eigen::MatrixXd>(e_hat.data(), 3, 3);
    svd = Eigen::JacobiSVD<Eigen::MatrixXd>(E_hat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd U = svd.matrixU();
    V = svd.matrixV();
    Eigen::MatrixXd _value = svd.singularValues();
    Eigen::MatrixXd singularValues(Eigen::MatrixXd::Zero(3, 3));;
    singularValues(0, 0) = _value(0);
    singularValues(1, 1) = _value(1);
    E = points2_norm_matrix.transpose() * U * singularValues * V.transpose() * points1_norm_matrix;
    return E;
}

Eigen::Matrix3d EssentialMatrixFromPose(const Eigen::Matrix3d& R,
                                        const Eigen::Vector3d& t) {
    Eigen::Matrix3d E;
    E.setIdentity();
    // homework3
    Eigen::Matrix3d tx;
    tx << 0, -t(2), t(1),
            t(2), 0, -t(0),
            -t(1), t(0), 0;
    E = tx * R;
    return E;
}

