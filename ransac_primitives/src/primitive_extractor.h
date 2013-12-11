#ifndef PRIMITIVE_EXTRACTOR_H
#define PRIMITIVE_EXTRACTOR_H

#include <pcl/point_cloud.h>
#include <pcl/octree/octree_impl.h>
#include <Eigen/Dense>
#include "primitive_params.h"
#include "primitive_octree.h"
#include "base_primitive.h"
#include "primitive_visualizer.h"

class primitive_extractor {
public:
    typedef pcl::PointXYZRGB point;
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud; // pcl formats
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
    Eigen::MatrixXd mpoints; // points of cloud
    Eigen::MatrixXd mnormals; // normals of point cloud
    primitive_octree octree; // octree used for the entire cloud
    int tree_depth; // the levels of the octree
    Eigen::ArrayXi level_scores; // the sum of the number of primitives at each level of the octree
    //Eigen::VectorXd level_pdf;
    std::vector<base_primitive*> candidates; // candidates for new primitives

    std::vector<base_primitive*>& primitives; // primitive types used

    primitive_params params;
    double inlier_threshold; // distance from shape that counts as inlier
    double angle_threshold; // highest allowed normal angle deviation
    double add_threshold; // probability required to add shape, P(|m| | |C|) < add_threshold
    int least_support; // stop when we're confident enough not to have overlooked a shape with least_support points
    double normal_radius; // radius within which we'll estimate the normals of a point

    primitive_visualizer* vis;

    void remove_distant_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud, double dist);
    void estimate_normals();
    int sample_level(int iteration);
    void get_points_at_level(std::vector<int>& inds, point& p, int level);
    void generate_random_subsets(std::vector<std::vector<int> >& subsets, int n);
    double prob_candidate_not_found(double candidate_size,
                                    double candidates_evaluated,
                                    int points_required);
    void remove_points_from_cloud(const std::vector<int>& ind, base_primitive::shape shape);
    void add_new_primitive(base_primitive* primitive);
public:
    void primitive_inlier_points(Eigen::MatrixXd& points, base_primitive* p);
    void extract(std::vector<base_primitive*>& extracted);
    pcl::PointCloud<pcl::Normal>::ConstPtr get_normals();
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr get_cloud();
    primitive_extractor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                        std::vector<base_primitive*>& primitives,
                        primitive_params params = primitive_params(),
                        primitive_visualizer* vis = NULL);
};

#endif // PRIMITIVE_EXTRACTOR_H
