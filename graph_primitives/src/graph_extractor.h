#ifndef GRAPH_EXTRACTOR_H
#define GRAPH_EXTRACTOR_H

#include <Eigen/Dense>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "base_primitive.h"

class graph_extractor
{
public:
    typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
            boost::property<boost::vertex_color_t, boost::default_color_type>,
            boost::property<boost::edge_weight_t, int>
            > graph;
protected:
    graph g;
    double adjacency_dist;
    std::vector<base_primitive*> primitives;
    void construct_adjacency_graph(std::vector<Eigen::MatrixXd>& inliers);
    double primitive_distance(Eigen::MatrixXd& inliers1, Eigen::MatrixXd& inliers2);
public:
    void generate_dot_file(const std::string& filename);
    graph_extractor(const std::vector<base_primitive*>& primitives, std::vector<Eigen::MatrixXd>& inliers, double adjacency_dist);
};

#endif // GRAPH_EXTRACTOR_H
