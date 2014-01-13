#ifndef CYLINDER_PRIMITIVE_H
#define CYLINDER_PRIMITIVE_H

#include "base_primitive.h"

#include <Eigen/Dense>

class cylinder_primitive : public base_primitive
{
private:
    Eigen::Vector3d a;
    Eigen::Vector3d c;
    double r;
    double h;
    Eigen::Matrix<double, 3, 2> basis;
    double max_radius;
    static int cylinders_drawn;
    Eigen::Vector2d min2;
    Eigen::Vector2d max2;
public:
    bool construct(const Eigen::MatrixXd& points, const Eigen::MatrixXd& normals,
                   double inlier_threshold, double angle_threshold);
    int inliers(const Eigen::MatrixXd& points, const Eigen::MatrixXd& normals, const std::vector<int>& inds,
                double inlier_threshold, double angle_threshold);
    int points_required();
    double distance_to_pt(const Eigen::Vector3d& pt);
    void direction_and_center(Eigen::Vector3d& direction, Eigen::Vector3d& center);
    void compute_shape_size(const Eigen::MatrixXd& points);
    double shape_size();
    double shape_data(Eigen::VectorXd& data);
    shape get_shape();
    void draw(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
    base_primitive* instantiate();
    cylinder_primitive();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // CYLINDER_PRIMITIVE_H
