#include "primitive_extractor.h"

#include "plane_primitive.h"
#include "sphere_primitive.h"
#include "cylinder_primitive.h"

#include <iomanip>
#include <time.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
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
    base_primitive::number_disjoint_subsets = params.number_disjoint_subsets;
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
    construct_octrees();

    std::cout << "Octree constructed, tree depth: " << tree_depth << std::endl;

    // estimate normals for all points
    estimate_normals();

    std::cout << "Normals extracted..." << std::endl;

    mpoints.resize(3, cloud->size());
    mnormals.resize(3, cloud->size());

    for (int i = 0; i < cloud->size(); ++i) {
        //cloud->points[i].r = 255;
        mpoints.col(i) = cloud->points[i].getVector3fMap().cast<double>();
        mnormals.col(i) = cloud_normals->points[i].getNormalVector3fMap().cast<double>();
        mnormals.col(i).normalize();
    }
}

void primitive_extractor::construct_octrees()
{
    std::vector<int> inds;
    inds.resize(cloud->size());
    for (int i = 0; i < cloud->size(); ++i) {
        inds[i] = i;
    }
    std::random_shuffle(inds.begin(), inds.end());

    // setup the disjoint point sets
    octrees.resize(params.number_disjoint_subsets, primitive_octree(params.octree_res)); // could probably just pass an int?
    total_set_size.resize(params.number_disjoint_subsets);
    int sum = 0;

    int step = int(cloud->size())/params.number_disjoint_subsets; // assuming A >> B
    for (int i = 0; i < params.number_disjoint_subsets; ++i) {
        std::vector<int>::iterator start = inds.begin() + i*step;
        std::vector<int>::iterator end = inds.begin() + (i+1)*step;
        if (i == params.number_disjoint_subsets - 1) {
            end = inds.end();
        }
        octrees[i].setInputCloud(cloud, primitive_octree::IndicesConstPtr(new std::vector<int>(start, end)));
        octrees[i].addPointsFromInputCloud();
        sum += octrees[i].size();
        total_set_size[i] = sum;
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
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
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
    number_extracted = 0; // set numbers extracted to zero again
    extracted.clear(); // extracted primitives

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

    double prob_not_found = 1.0;
    std::vector<int> inds;
    int iteration = 0;
    do {
        // pick one point from entire cloud
        int ind = rand() % n; // change to work for clouds > RAND_MAX
        if (isnan(cloud->points[ind].x) || isnan(cloud->points[ind].y) || isnan(cloud->points[ind].z)) {
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
                c->refine_inliers(octrees, mpoints, mnormals, params.inlier_threshold, params.angle_threshold);
                level_scores(level) += c->supporting_inds.size(); // TODO: this must be expected value instead
                // should this get updated when intervals are refined? should be easy
            }
            else {
                delete c;
            }
        }
        candidates_evaluated += 1.0;

        // no candidates -> can't do anything
        if (candidates.size() == 0) {
            ++iteration;
            continue;
        }

        int rounds = 0;
        std::vector<base_primitive*> best_candidates = candidates;
        double best_val;
        base_primitive* last_candidate = NULL;
        //int repeat = 0;
        do {
            best_val = refine_inliers(best_candidates);
            if (best_candidates.size() == 1 && best_candidates[0] != last_candidate) {
                best_candidates[0]->final_inliers(octree, mpoints, mnormals, params.inlier_threshold, params.angle_threshold);
                last_candidate = best_candidates[0];
                best_candidates = candidates;
                /*if (repeat > 0) {
                    std::cout << "Repeat: " << repeat << std::endl;
                }
                ++repeat;*/
            }
            ++rounds;
        }
        while (best_candidates.size() > 1);

        base_primitive* best_candidate = best_candidates[0];
        if (PRINTOUTS) {
            std::cout << "Candidates: " << candidates.size() << std::endl;
            std::cout << "Extracted: " << extracted.size() << std::endl;
            std::cout << "Points left: " << octree.size() << std::endl;
            std::cout << "Best val:" << best_val << std::endl;
            std::cout << "Best cand prob: " << std::setprecision(10) << prob_candidate_not_found(double(best_val), candidates_evaluated,
                                                                        best_candidate->points_required()) << std::endl;
            std::cout << "Rounds: " << rounds << std::endl;
        }

        // if no better candidate can be found with P > 1 - add_threshold -> add to extracted, remove overlapping from candidates
        if (prob_candidate_not_found(best_val, candidates_evaluated, min_set) < params.add_threshold) {
            // here we remove the points in the octree that are contained in best_candidate
            //add_new_primitive(best_candidate); // this removes the points from the octree before re-calculating candidates_evaluated
            if (best_candidate->refinement_level() < params.number_disjoint_subsets) {
                best_candidate->final_inliers(octree, mpoints, mnormals, params.inlier_threshold, params.angle_threshold);
            }
            extracted.push_back(best_candidate);
            std::vector<base_primitive*> keep_candidates; // candidate to keep
            keep_candidates.reserve(candidates.size());
            for (base_primitive* p : candidates) {
                if (p == best_candidate) {
                    // do nothing
                }
                else if (p->are_contained(best_candidate->sorted_inliers())) {
                    // remove candidate
                    candidates_evaluated *= pow(1 - double(p->get_inliers())/double(octree.size()), 3.0);
                    delete p;
                }
                else {
                    // keep in new vector
                    keep_candidates.push_back(p);
                }
            }
            add_new_primitive(best_candidate);
            candidates.swap(keep_candidates);
            keep_candidates.clear();
        }

        prob_not_found = prob_candidate_not_found(params.min_shape, candidates_evaluated, min_set);
        if (std::isinf(prob_not_found)) {
            clear_primitives(extracted);
            break;
        }

        if (PRINTOUTS) {
            std::cout << "Prob min cand not found: " << prob_not_found << std::endl;
        }
        ++iteration;
    }
    while (prob_not_found > params.add_threshold);

    // min_set because that will be the most unlikely shape
    clear_primitives(candidates);

    // compute the sizes of the extracted primitives
    for (base_primitive* p : extracted) {
        p->compute_shape_size(mpoints);
    }
}

base_primitive* primitive_extractor::max_inliers(double& maxmean, double& maxa, double& maxb,
                                                 std::vector<base_primitive*>& primitives)
{
    base_primitive* best_candidate;
    maxmean = -INFINITY;
    double mean, a, b;
    for (base_primitive* p : primitives) {
        p->inliers_estimate(mean, a, b, octree.size(), total_set_size);
        if (mean > maxmean) {
            maxmean = mean;
            maxa = a;
            maxb = b;
            best_candidate = p;
        }
    }
    return best_candidate;
}

void primitive_extractor::overlapping_estimates(std::vector<base_primitive*>& primitives, base_primitive* best_candidate)
{
    double maxmean, maxa, maxb;
    best_candidate->inliers_estimate(maxmean, maxa, maxb, octree.size(), total_set_size);
    std::vector<base_primitive*> temp;
    temp.push_back(best_candidate);
    double mean, a, b;
    for (base_primitive* p : primitives) {
        if (p == best_candidate) {
            continue;
        }
        p->inliers_estimate(mean, a, b, octree.size(), total_set_size);
        // check if confidence intervals overlap
        if ((a < maxa && maxa < b) || (a < maxb && maxb < b) ||
                (maxa < a && a < maxb) || (maxa < b && b < maxb)) {
            temp.push_back(p);
        }
    }
    primitives.swap(temp);
}

double primitive_extractor::refine_inliers(std::vector<base_primitive*>& primitives)
{
    // find the candidate with the most expected inliers
    double maxmean, maxa, maxb;
    base_primitive* best_candidate = max_inliers(maxmean, maxa, maxb, primitives);
    //best_candidate->final_inliers(octree, mpoints, mnormals, params.inlier_threshold, params.angle_threshold);
    //best_candidate->inliers_estimate(maxmean, maxa, maxb, octree.size(), total_set_size);
    int max_refinement = best_candidate->refinement_level();

    // find all candidates with overlapping bounds, discard the rest
    overlapping_estimates(primitives, best_candidate);

    /*if (primitives.size() == 1) {
        best_candidate->final_inliers(octree, mpoints, mnormals, params.inlier_threshold, params.angle_threshold);
        primitives = candidates;
        best_candidate = max_inliers(maxmean, maxa, maxb, primitives);
        overlapping_estimates(primitives, maxa, maxb);
        int max_refinement = best_candidate->refinement_level();
    }*/

    // refine if there is more than one found
    // TODO: do this in a smarter way
    if (primitives.size() == 1) {
        return maxmean;
    }
    for (base_primitive* p : primitives) {
        if (p->refinement_level() <= max_refinement) {
            p->refine_inliers(octrees, mpoints, mnormals, params.inlier_threshold, params.angle_threshold);
        }
    }

    return maxmean;
}

void primitive_extractor::clear_primitives(std::vector<base_primitive*>& ps)
{
    for (base_primitive* p : ps) {
        delete p;
    }
    ps.clear();
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
    int sum = 0;
    //std::cout << "Octree sizes: ";
    for (int i = 0; i < params.number_disjoint_subsets; ++i) {
        //std::cout << octrees[i].size() << " ";
        octrees[i].remove_points(p->supporting_inds);
        sum += octrees[i].size();
        total_set_size[i] = sum;
    }
    std::cout << std::endl;

    if (vis != NULL) {
        vis->lock();
    }

    int colormap[6][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 0, 255}, {255, 255, 0}, {64, 224, 208}};
    int r = number_extracted % 6;
    p->red = colormap[r][0];
    p->green = colormap[r][1];
    p->blue = colormap[r][2];
    ++number_extracted;

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
