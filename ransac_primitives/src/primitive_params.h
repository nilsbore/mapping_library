#ifndef PRIMITIVE_PARAMS_H
#define PRIMITIVE_PARAMS_H

struct primitive_params
{
    double octree_res;
    double normal_neigbourhood;
    double inlier_threshold;
    double angle_threshold;
    double add_threshold;
    int min_shape;
    int inlier_min;
    double connectedness_res;
    double distance_threshold;
    primitive_params(const primitive_params& other)
    {
        octree_res = other.octree_res;
        normal_neigbourhood = other.normal_neigbourhood;
        inlier_threshold = other.inlier_threshold;
        angle_threshold = other.angle_threshold;
        add_threshold = other.add_threshold;
        min_shape = other.min_shape;
        inlier_min = other.inlier_min;
        connectedness_res = other.connectedness_res;
        distance_threshold = other.distance_threshold;
    }

    primitive_params()
    {
        octree_res = 0.2;
        normal_neigbourhood = 0.02;
        inlier_threshold = 0.01;
        angle_threshold = 0.3;
        add_threshold = 0.05;
        min_shape = 1000;
        inlier_min = min_shape;
        connectedness_res = 0.02;
        distance_threshold = 2.0;
    }
};

#endif // PRIMITIVE_PARAMS_H
