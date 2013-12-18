#ifndef GRAPH_EXTRACTOR_H
#define GRAPH_EXTRACTOR_H

#include <Eigen/Dense>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "base_primitive.h"
#include "conversions.h"

class primitive_label_writer {
public:
    primitive_label_writer(std::vector<base_primitive*>& primitives) : primitives(primitives) {}
    template <class VertexOrEdge>
    void operator()(std::ostream& out, const VertexOrEdge& v) const {
        std::string name;
        std::string shape;
        base_primitive* p = primitives[v];
        std::cout << "Shape: " << p->get_shape() << std::endl;
        switch (p->get_shape()) {
        case base_primitive::SPHERE:
            name = "Sphere";
            shape = "ellipse";
            break;
        case base_primitive::PLANE:
            name = "Plane";
            shape = "box";
            break;
        case base_primitive::CYLINDER:
            name = "Cylinder";
            shape = "ellipse";
            break;
        default:
            break;
        }
        out << "[label=\"" << name << "\"]";
        out << "[shape=\"" << shape << "\"]";
        out << "[style=\"filled\"]";
        out << "[fillcolor=\"" << convenience::rgb_to_hex_string(p->red, p->green, p->blue) << "\"]";
    }
private:
    std::vector<base_primitive*>& primitives;
};

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
