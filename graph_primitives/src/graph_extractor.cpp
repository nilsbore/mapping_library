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
    for (int i = 1; i < inliers.size(); ++i) {
        for (int j = 0; j < i; ++j) {
            mindist = primitive_distance(inliers[i], inliers[j]);
            std::cout << "Min dist for " << i << " and " << j << " is " << mindist << std::endl;
            if (mindist < adjacency_dist) {
                boost::add_edge(v[i], v[j], g);
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
        //MatrixXd temp2 = temp.colwise().squaredNorm();
        //std::cout << "temp2 rows: " << temp2.rows() << ", cols: " << temp2.cols() << std::endl;
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
    boost::write_graphviz(file, g, writer);//boost::make_label_writer(name)); // dot -Tpng test2.dot > test2.png
    file.close();
}
