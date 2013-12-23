#ifndef PLANE_PRIMITIVE_H
#define PLANE_PRIMITIVE_H

#include "base_primitive.h"

#include <Eigen/Dense>

class plane_primitive : public base_primitive
{
private:
    Eigen::Vector4d p;
    Eigen::Matrix<double, 3, 2> basis;
public:
    bool construct(const Eigen::MatrixXd& points, const Eigen::MatrixXd& normals,
                   double inlier_threshold, double angle_threshold);
    int inliers(const Eigen::MatrixXd& points, const Eigen::MatrixXd& normals, const std::vector<int>& inds,
                double inlier_threshold, double angle_threshold);
    int points_required();
    double distance_to_pt(const Eigen::Vector3d& pt);
    void direction_and_center(Eigen::Vector3d& direction, Eigen::Vector3d& center);
    double shape_size();
    shape get_shape();
    void draw(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
    base_primitive* instantiate();
    plane_primitive();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // PLANE_PRIMITIVE_H
