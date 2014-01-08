#include "graph_extractor.h"

#include <boost/graph/graphviz.hpp>
#include <fstream>

using namespace Eigen;

graph_extractor::graph_extractor(const std::vector<base_primitive*>& primitives, std::vector<MatrixXd>& inliers, double adjacency_dist) :
    primitives(primitives), adjacency_dist(adjacency_dist)
{
    std::vector<MatrixXd> sparse_inliers;
    sparse_inliers.resize(inliers.size());
    int skip = 10;
    for (int i = 0; i < inliers.size(); ++i) {
        int newsize = int(inliers[i].cols()) / skip;
        sparse_inliers[i].resize(3, newsize);
        for (int j = 0; j < newsize; ++j) {
            sparse_inliers[i].col(j) = inliers[i].col(skip*j);
        }
    }
    construct_adjacency_graph(sparse_inliers);
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
    std::vector<Vector3d, aligned_allocator<Vector3d> > trans;
    for (int i = 1; i < inliers.size(); ++i) {
        for (int j = 0; j < i; ++j) {
            trans.clear();
            bool are_planes = primitives[i]->get_shape() == base_primitive::PLANE && primitives[j]->get_shape() == base_primitive::PLANE;
            bool is_sphere = primitives[i]->get_shape() == base_primitive::SPHERE || primitives[j]->get_shape() == base_primitive::SPHERE;
            mindist = primitive_distance(trans, inliers[i], inliers[j], are_planes);
            std::cout << "Min dist for " << i << " and " << j << " is " << mindist << std::endl;
            if (mindist < adjacency_dist) {
                primitives[i]->direction_and_center(d1, c1);
                primitives[j]->direction_and_center(d2, c2);
                edge_weight_property e;
                if (is_sphere) {
                    e = 0;
                }
                else if (are_planes) {
                    e = plane_angle(trans, d1, d2);
                }
                else {
                    e = acos(fabs(d1.dot(d2)));
                }
                std::pair<boost::graph_traits<graph>::edge_descriptor, bool> result = boost::add_edge(v[i], v[j], e, g);
            }
        }
    }

}

double graph_extractor::plane_angle(const std::vector<Vector3d, aligned_allocator<Vector3d> >& trans, const Vector3d& d1, const Vector3d& d2)
{
    int s = 0;
    for (const Vector3d& t : trans) {
        if ((d1 - d2).dot(t) > 0) { // planes facing each other (think normal geometry)
            s += 1;
        }
        else { // outwarding facing edge, same reasoning
            s -= 1;
        }
    }
    std::cout << "Sum: " << s << std::endl;
    std::cout << "Total: " << trans.size() << std::endl;
    double angle = M_PI;
    if (s > 0) {
        angle -= acos(d1.dot(d2));
    }
    else {
        angle += acos(d1.dot(d2));
    }
    return angle;
}

double graph_extractor::primitive_distance(std::vector<Vector3d, aligned_allocator<Vector3d> >& trans,
                                           const MatrixXd& inliers1, const MatrixXd& inliers2, bool are_planes)
{
    double mindist = INFINITY;
    MatrixXd temp;
    double mincol;
    int index;
    for (int i = 0; i < inliers1.cols(); ++i) {
        temp = inliers1.col(i).replicate(1, inliers2.cols());
        temp -= inliers2;
        mincol = temp.colwise().squaredNorm().minCoeff(&index);
        if (are_planes && sqrt(mincol) < adjacency_dist) {
            trans.push_back(inliers2.col(index) - inliers1.col(i));
        }
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
