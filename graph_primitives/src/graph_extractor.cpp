#include "graph_extractor.h"

#include <boost/graph/graphviz.hpp>
#include <fstream>

using namespace Eigen;

graph_extractor::graph_extractor(const std::vector<base_primitive*>& primitives, std::vector<MatrixXd>& inliers, double adjacency_dist) :
    primitives(primitives), adjacency_dist(adjacency_dist)
{
    construct_adjacency_graph(inliers);
}

void graph_extractor::construct_adjacency_graph(std::vector<MatrixXd>& inliers)
{
    std::vector<graph::vertex_descriptor> v;
    v.resize(primitives.size());
    for (int i = 0; i < primitives.size(); ++i) {
        v[i] = boost::add_vertex(g);
    }
    double mindist;
    Vector3d d1;
    Vector3d c1;
    Vector3d d2;
    Vector3d c2;
    for (int i = 1; i < inliers.size(); ++i) {
        for (int j = 0; j < i; ++j) {
            mindist = primitive_distance(inliers[i], inliers[j]);
            std::cout << "Min dist for " << i << " and " << j << " is " << mindist << std::endl;
            if (mindist < adjacency_dist) {
                primitives[i]->direction_and_center(d1, c1);
                primitives[j]->direction_and_center(d2, c2);
                edge_weight_property e = acos(fabs(d1.dot(d2)));
                std::pair<boost::graph_traits<graph>::edge_descriptor, bool> result = boost::add_edge(v[i], v[j], e, g);
            }
        }
    }

}

double graph_extractor::primitive_distance(MatrixXd& inliers1, MatrixXd& inliers2)
{
    double mindist = INFINITY;
    MatrixXd temp;
    double mincol;
    int skip = 100;
    for (int i = 0; i < inliers1.cols(); i += skip) {
        temp = inliers1.col(i).replicate(1, inliers2.cols());
        temp -= inliers2;
        mincol = temp.colwise().squaredNorm().minCoeff();
        if (sqrt(mincol) < mindist) {
            mindist = sqrt(mincol);
        }
    }
    return mindist;
}

void graph_extractor::generate_dot_file(const std::string& filename)
{
    std::ofstream file;
    file.open(filename);
    primitive_label_writer writer(primitives);
    primitive_edge_writer<graph> edge_writer(g);
    boost::write_graphviz(file, g, writer, edge_writer);//boost::make_label_writer(name)); // dot -Tpng test2.dot > test2.png
    file.close();
}

void graph_extractor::generate_index_file(const std::string& filename)
{
    std::ofstream file;
    file.open(filename);
    for (base_primitive* p : primitives) {
        p->write_indices_to_stream(file);
        file << "\n";
    }
    file.close();
}
