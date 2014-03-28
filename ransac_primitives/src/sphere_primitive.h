#ifndef SPHERE_PRIMITIVE_H
#define SPHERE_PRIMITIVE_H

#include "base_primitive.h"

#include <Eigen/Dense>

class sphere_primitive : public base_primitive
{
private:
    Eigen::Vector3d c;
    double r;
    double max_radius;
    static int spheres_drawn;
    bool sphere_to_grid(Eigen::Vector2d& gridpt, const Eigen::Vector3d& spherept);
public:
    bool construct(const Eigen::MatrixXd& points, const Eigen::MatrixXd& normals,
                   double inlier_threshold, double angle_threshold);
    void compute_inliers(std::vector<int>& inliers, const Eigen::MatrixXd& points, const Eigen::MatrixXd& normals,
                         const std::vector<int>& inds, double inlier_threshold, double angle_threshold);
    void largest_connected_component(std::vector<int>& inliers, const Eigen::MatrixXd& points);
    int points_required();
    double distance_to_pt(const Eigen::Vector3d& pt);
    void direction_and_center(Eigen::Vector3d& direction, Eigen::Vector3d& center);
    double shape_size();
    double shape_data(Eigen::VectorXd& data);
    void shape_points(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& points);
    shape get_shape();
    void draw(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
    base_primitive* instantiate();
    sphere_primitive();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // SPHERE_PRIMITIVE_H
