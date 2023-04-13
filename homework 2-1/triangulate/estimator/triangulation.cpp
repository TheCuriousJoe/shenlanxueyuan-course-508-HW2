#include "triangulation.h"
#include <iostream>

Eigen::Vector3d TriangulatePoint(const Eigen::Matrix3x4d& proj_matrix1,
                                 const Eigen::Matrix3x4d& proj_matrix2,
                                 const Eigen::Vector2d& point1,
                                 const Eigen::Vector2d& point2) {
  // homework1
  Eigen::MatrixXd A(Eigen::MatrixXd::Zero(4, 4));
  Eigen::MatrixXd m1_r1 = proj_matrix1.row(0);
  Eigen::MatrixXd m1_r2 = proj_matrix1.row(1);
  Eigen::MatrixXd m1_r3 = proj_matrix1.row(2);
  A.row(0) = point1.x() * proj_matrix1.row(2) - proj_matrix1.row(0);
  A.row(1) = point1.y() * proj_matrix1.row(2) - proj_matrix1.row(1);
  A.row(2) = point2.x() * proj_matrix2.row(2) - proj_matrix2.row(0);
  A.row(3) = point2.y() * proj_matrix2.row(2) - proj_matrix2.row(1);

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
  Eigen::MatrixXd V = svd.matrixV();
//  std::cout << "V is: " << std::endl << V << std::endl;
  Eigen::Vector4d m = V.col(V.cols() - 1);
  Eigen::Vector3d point(m(0)/m(3), m(1)/m(3), m(2)/m(3));
//  std::cout << "point is: " << std::endl << point << std::endl;
  return point;

}

std::vector<Eigen::Vector3d> TriangulatePoints(
    const Eigen::Matrix3x4d& proj_matrix1,
    const Eigen::Matrix3x4d& proj_matrix2,
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2) {
  // homework2
  std::vector<Eigen::Vector3d> points3D;
  for (int i = 0; i < points1.size(); i++){
    Eigen::Vector3d point = TriangulatePoint(proj_matrix1, proj_matrix2, points1[i], points2[i]);
    points3D.push_back(point);
  }
  return points3D;
}
