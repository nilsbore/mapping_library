#include "primitive_extractor.h"

#include "plane_primitive.h"
#include "sphere_primitive.h"
#include "cylinder_primitive.h"

#include <iomanip>
#include <time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>

#define PRINTOUTS false

using namespace Eigen;

primitive_extractor::primitive_extractor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud,
                                         std::vector<base_primitive*>& primitives,
                                         primitive_params params, primitive_visualizer* vis) :
    cloud(new pcl::PointCloud<pcl::PointXYZRGB>()), cloud_normals(new pcl::PointCloud<pcl::Normal>),
    octree(params.octree_res), primitives(primitives), params(params), vis(vis)
{
    // setup parameters
    base_primitive::min_inliers = params.inlier_min;
    base_primitive::connectedness_res = params.connectedness_res;

    // seed the random number generator
    srand(time(NULL));

    // create the point cloud from the points that are close enough
    remove_distant_points(new_cloud, params.distance_threshold);

    // initialize clouds and matrices
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();
    tree_depth = octree.getTreeDepth() + 1;
    level_scores.resize(tree_depth);
    level_scores.setZero();

    // estimate normals for all points
    estimate_normals();

    std::cout << "Tree depth: " << tree_depth << std::endl;

    mpoints.resize(3, cloud->size());
    mnormals.resize(3, cloud->size());

    for (int i = 0; i < cloud->size(); ++i) {
        mpoints.col(i) = cloud->points[i].getVector3fMap().cast<double>();
        mnormals.col(i) = cloud_normals->points[i].getNormalVector3fMap().cast<double>();
        mnormals.col(i).normalize();
    }
}

pcl::PointCloud<pcl::Normal>::ConstPtr primitive_extractor::get_normals()
{
    return cloud_normals;
}

pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr primitive_extractor::get_cloud()
{
    return cloud;
}

void primitive_extractor::remove_distant_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud, double dist)
{
    // filter out points that are far from the camera and thus will contain too much noise
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(new_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, dist);
    pass.filter(*cloud);
}

void primitive_extractor::estimate_normals()
{
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    //pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr ptr(octree);
    ne.setSearchMethod(tree);

    // Use all neighbors in a sphere of radius normal_radius m
    ne.setRadiusSearch(params.normal_neigbourhood);

    // Compute the features
    ne.compute(*cloud_normals);

    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
}

void primitive_extractor::extract(std::vector<base_primitive*>& extracted)
{
    extracted.clear(); // extracted primitives
    int counter; // to use where we need it

    // loop through all types and find maximum points required to construct shape
    int min_set = 1;
    for (base_primitive* p : primitives) {
        if (p->points_required() > min_set) {
            min_set = p->points_required();
        }
    }
    std::cout << "Minimum required: " << min_set << std::endl;

    int n = cloud->size();
    double candidates_evaluated = 0.0;

    std::vector<int> inds;
    int iteration = 0;
    do {
        // pick one point from entire cloud
        int ind = rand() % n; // change to work for clouds > RAND_MAX
        if (isnan(cloud->points[ind].x)) {
            continue;
        }

        // random tree depth, sample more points from that depth at point
        int level = sample_level(iteration);

        // sample enough points to extract all shapes
        MatrixXd points(3, min_set);
        MatrixXd normals(3, min_set);
        points.col(0) = mpoints.col(ind);
        normals.col(0) = mnormals.col(ind);

        // get points in same cell as first point at sampled depth
        inds.clear();
        get_points_at_level(inds, cloud->points[ind], level);
        if (inds.empty()) {
            continue;
        }

        // no need to check uniqueness, will not work well anyways
        for (int i = 1; i < min_set; ++i) {
            int ind = inds[rand() % inds.size()];
            points.col(i) = mpoints.col(ind);
            normals.col(i) = mnormals.col(ind);
        }

        // iterate over possible shapes, evaluate them to see if points good -> count inliers
        for (base_primitive* p : primitives) {
            base_primitive* c = p->instantiate();
            if (c->construct(points, normals, params.inlier_threshold, params.angle_threshold)) {
                candidates.push_back(c);
                inds.clear();
                octree.find_potential_inliers(inds, c, params.inlier_threshold + 0.01);
                c->inliers(mpoints, mnormals, inds, params.inlier_threshold, params.angle_threshold);
                level_scores(level) += c->get_inliers();
            }
            else {
                delete c;
            }
        }
        candidates_evaluated += 1.0;

        // no candidates -> can't do anything
        if (candidates.size() == 0) {
            continue;
        }

        // find the candidate with the most inliers
        int best_val = -1;
        int best_ind;
        counter = 0;
        for (base_primitive* p : candidates) {
            if (p->get_inliers() > best_val) {
                best_val = p->get_inliers();
                best_ind = counter;
            }
            ++counter;
        }

        base_primitive* best_candidate = candidates[best_ind];
        if (PRINTOUTS) {
            std::cout << "Candidates: " << candidates.size() << std::endl;
            std::cout << "Extracted: " << extracted.size() << std::endl;
            std::cout << "Points left: " << octree.size() << std::endl;
            std::cout << "Best val:" << best_val << std::endl;
            std::cout << "Best cand prob: " << std::setprecision(10) << prob_candidate_not_found(double(best_val), candidates_evaluated,
                                                                        best_candidate->points_required()) << std::endl;
        }

        // if no better candidate can be found with P > 1 - add_threshold -> add to extracted, remove overlapping from candidates
        if (prob_candidate_not_found(best_val, candidates_evaluated, min_set) < params.add_threshold) {
            // here we remove the points in the octree that are contained in best_candidate
            add_new_primitive(best_candidate);
            extracted.push_back(best_candidate);
            std::vector<base_primitive*> keep_candidates; // candidate to keep
            keep_candidates.reserve(candidates.size());
            for (base_primitive* p : candidates) {
                if (p == best_candidate) {
                    // do nothing
                }
                else if (p->are_contained(best_candidate->supporting_inds)) {
                    // remove candidate
                    candidates_evaluated = pow(1 - double(p->get_inliers())/double(octree.size()), 3.0)*candidates_evaluated;
                    delete p;
                }
                else {
                    // keep in new vector
                    keep_candidates.push_back(p);
                }
            }
            candidates.swap(keep_candidates);
            keep_candidates.clear();
        }

        if (PRINTOUTS) {
            std::cout << "Prob min cand not found: " << prob_candidate_not_found(params.min_shape, candidates_evaluated, min_set) << std::endl;
        }
        ++iteration;
    }
    while (prob_candidate_not_found(params.min_shape, candidates_evaluated, min_set) > params.add_threshold);
    // min_set because that will be the most unlikely shape

}

void primitive_extractor::add_new_primitive(base_primitive* primitive)
{
    if (vis != NULL) {
        vis->lock();
        primitive->draw(vis->viewer);
        vis->unlock();
    }
    remove_points_from_cloud(primitive);
}

void primitive_extractor::remove_points_from_cloud(base_primitive* p)
{
    octree.remove_points(p->supporting_inds); // remove points in octree that are part of shape

    if (vis != NULL) {
        vis->lock();
    }

    p->red = 0;
    p->green = 0;
    p->blue = 0;

    int r = rand() % 5;
    if (r == 0) {
        p->green = 255;
    }
    else if (r == 1) {
        p->red = 255;
        p->blue = 255;
    }
    else if (r == 2) {
        p->green = 255;
        p->blue = 255;
    }
    else if (r == 3) {
        p->red = 255;
    }
    else {
        p->blue = 255;
    }

    /*switch (p->get_shape()) {
    case base_primitive::PLANE:
        p->red = 255;
        break;
    case base_primitive::SPHERE:
        p->green = 255;
        break;
    case base_primitive::CYLINDER:
        p->blue = 255;
        break;
    default:
        break;
    }*/
    for (const int& i : p->supporting_inds) {
        cloud->points[i].r = p->red;
        cloud->points[i].g = p->green;
        cloud->points[i].b = p->blue;
    }

    if (vis != NULL) {
        vis->cloud_changed = true;
        vis->unlock();
    }
}

int primitive_extractor::sample_level(int iteration)
{
    if (iteration < 200) {
        return rand() % tree_depth;
    }
    // rejection sampling approach
    double x = 0.9;
    double weight = double(level_scores.sum());
    ArrayXd pdf = x/weight*level_scores.cast<double>() + (1-x)*1/double(tree_depth);
    double maxval = pdf.maxCoeff();
    if (PRINTOUTS) {
        std::cout << "Pdf: " << pdf.transpose() << std::endl;
        std::cout << "Level scores: " << level_scores.transpose() << std::endl;
    }
    int i;
    double d;
    while (true) {
        i = rand() % tree_depth;
        d = double(rand())/double(RAND_MAX);
        if (pdf[i] > d*maxval) {
            return i;
        }
    }
}

void primitive_extractor::get_points_at_level(std::vector<int>& inds, point& p, int level)
{
    octree.find_points_at_depth(inds, p, level);
}

void primitive_extractor::generate_random_subsets(std::vector<std::vector<int> >& subsets, int n)
{
    subsets.resize(n);
    for (int i = 0; i < n; ++i) {
        subsets[i].reserve(cloud->size() / (n - 1)); // to be sure there is enough space
    }
    for (int i = 0; i < cloud->size(); ++i) {
        subsets[rand() % n].push_back(i);
    }
}

// candidates_evaluated needs to be modified after points have been deleted, candidates added
//
double primitive_extractor::prob_candidate_not_found(double candidate_size,
                                                     double candidates_evaluated,
                                                     int points_required)
{
    double intpart = octree.size()*tree_depth*(1 << points_required);
    return pow(1.0f - candidate_size/intpart, candidates_evaluated);
}

void primitive_extractor::primitive_inlier_points(MatrixXd& points, base_primitive* p)
{
    unsigned sz = p->supporting_inds.size();
    points.resize(3, sz);
    for (int i = 0; i < sz; ++i) {
        points.col(i) = mpoints.col(p->supporting_inds[i]);
    }
}
