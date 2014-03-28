#ifndef BASE_PRIMITIVE_H
#define BASE_PRIMITIVE_H

#include <Eigen/Dense>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/visualization/pcl_visualizer.h>

class primitive_octree; // primitive_octree depends on base_primitive

class base_primitive
{
public:
    // the different kinds of primitives that we are treating, only used for coloring
    enum shape {PLANE, SPHERE, CYLINDER, TORUS, CONE};
    std::vector<int> supporting_inds; // the inliers to the primitive
    std::vector<int> conforming_inds;
    int inlier_refinement;
    bool connected_component_done;
    bool sorted;
    u_char red, green, blue; // colors used when displaying the primitive
    static int number_disjoint_subsets;
    static int min_inliers; // the minimum inliers required to check connectedness, skip primitive otherwise
    static double margin; // the margin used when discarding octree nodes by looking at the primitives
    static double connectedness_res; // the resolution in the discretized image where we check for connectedness
    int get_inliers() { return supporting_inds.size(); }
    // use the octree for this also, since they are all from one node... can do that as a previous step
    bool are_contained(const std::vector<int>& other_inds);
    int find_blobs(cv::Mat& label_image, bool wrap_height = false, bool wrap_sides = false);
    void circle_to_grid(Eigen::Vector2d& rtn, const Eigen::Vector2d onDisk);
    double current_connectedness_res();
    // output the indices in the point cloud contained in the primitive
    void write_indices_to_stream(std::ostream& o);
    // create a primitive, points_required points and normals needed for the operation
    virtual bool construct(const Eigen::MatrixXd& points, const Eigen::MatrixXd& normals,
                           double inlier_threshold, double angle_threshold) = 0;
    void refine_inliers(std::vector<primitive_octree>& octrees, Eigen::MatrixXd& points,
                        Eigen::MatrixXd& normals, double inlier_threshold, double angle_threshold);
    void final_inliers(primitive_octree& octree, Eigen::MatrixXd& points, Eigen::MatrixXd& normals,
                       double inlier_threshold, double angle_threshold);
    int refinement_level() const;
    virtual void largest_connected_component(std::vector<int>& inliers, const Eigen::MatrixXd& points) = 0;
    std::vector<int>& sorted_inliers();
    void inliers_estimate(double& mean, double& a, double& b, int set_size, std::vector<int>& total_set_size);
    // check for primitives, takes all points and normals considered and the indices that are still unoccupied by primitives
    virtual void compute_inliers(std::vector<int>& inliers, const Eigen::MatrixXd& points, const Eigen::MatrixXd& normals,
                                 const std::vector<int>& inds, double inlier_threshold, double angle_threshold) = 0;
    // the number of points needed for construct
    virtual int points_required() = 0;
    // check if the shape is on either side of xyz, the center of a box with side length l,
    // used for effective search for primitive inliers in an octree
    virtual double distance_to_pt(const Eigen::Vector3d& pt) = 0;
    // used to find the relative angles between primitives
    virtual void direction_and_center(Eigen::Vector3d& direction, Eigen::Vector3d& center) = 0;
    virtual double shape_size() = 0;
    virtual double shape_data(Eigen::VectorXd& data) = 0;
    virtual void shape_points(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& points) = 0;
    virtual void compute_shape_size(const Eigen::MatrixXd& points);
    // returns the shape type, only used for coloring atm
    virtual shape get_shape() = 0;
    // draw the shape in a pcl visualizer window
    virtual void draw(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer) = 0;
    // create a new primitive of the subclass
    virtual base_primitive* instantiate() = 0;
    virtual ~base_primitive() {}
    base_primitive() : inlier_refinement(0) {}
};

#endif // BASE_PRIMITIVE_H
