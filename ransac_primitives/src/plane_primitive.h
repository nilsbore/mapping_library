#ifndef PLANE_PRIMITIVE_H
#define PLANE_PRIMITIVE_H

#include "base_primitive.h"

#include <Eigen/Dense>

class plane_primitive : public base_primitive
{
private:
    Eigen::Vector4d p;
    Eigen::Matrix<double, 3, 2> basis;
    Eigen::Vector2d sizes;
    Eigen::Quaterniond quat;
    Eigen::Vector3d c;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > convex_hull;
protected:
    void find_smallest_enclosing_box(Eigen::Vector2d& cmin, Eigen::Matrix2d& axes,
                                     Eigen::Vector2d& lengths, std::vector<cv::Point>& pts);
public:
    bool construct(const Eigen::MatrixXd& points, const Eigen::MatrixXd& normals,
                   double inlier_threshold, double angle_threshold);
    int inliers(const Eigen::MatrixXd& points, const Eigen::MatrixXd& normals, const std::vector<int>& inds,
                double inlier_threshold, double angle_threshold);
    int points_required();
    double distance_to_pt(const Eigen::Vector3d& pt);
    void direction_and_center(Eigen::Vector3d& direction, Eigen::Vector3d& center);
    double shape_size();
    double shape_data(Eigen::VectorXd& data);
    void shape_points(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& points);
    void compute_shape_size(const Eigen::MatrixXd& points);
    shape get_shape();
    void draw(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
    base_primitive* instantiate();
    plane_primitive();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // PLANE_PRIMITIVE_H
