#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <dirent.h>
#include <iomanip>
#include <sstream>

#include "files.h"
#include "primitive_extractor.h"
#include "primitive_visualizer.h"
#include "plane_primitive.h"
#include "sphere_primitive.h"
#include "cylinder_primitive.h"
#include "graph_extractor.h"

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    //std::string dirname = "/home/nbore/catkin_ws/xtionclouds";
    //std::string dirname = "/home/nbore/Data/pcldata/Pcd";
    std::string dirname = "/home/nbore/Data/Primitives\ Forward/pcd";
    std::vector<std::string> files;
    if (!convenience::read_directory(files, dirname))
    {
        std::cout << "Directory empty..." << std::endl;
        return 0;
    }

    std::vector<base_primitive*> primitives = { new plane_primitive(), new cylinder_primitive(), new sphere_primitive() };
    primitive_params params;
    params.octree_res = 0.04;
    params.normal_neigbourhood = 0.02;
    params.inlier_threshold = 0.015;
    params.angle_threshold = 0.4;
    params.add_threshold = 0.01;
    params.min_shape = 4000;
    params.inlier_min = params.min_shape;
    params.connectedness_res = 0.02;
    params.distance_threshold = 2.0;

    primitive_visualizer viewer;
    viewer.create_thread();

    int counter = 0;
    for (const std::string& file : files) {

        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (file, *cloud) == -1)
        {
            std::cout << "Couldn't read file " << file << std::endl;
            return 0;
        }

        std::cout << "Processing " << file << "..." << std::endl;

        primitive_extractor pe(cloud, primitives, params, &viewer);
        viewer.cloud = pe.get_cloud();
        viewer.cloud_changed = true;
        viewer.cloud_normals = pe.get_normals();
        viewer.normals_changed = true;
        std::vector<base_primitive*> extracted;
        pe.extract(extracted);
        if (extracted.empty()) {
            continue;
        }

        std::vector<Eigen::MatrixXd> inliers;
        inliers.resize(extracted.size());
        for (int i = 0; i < extracted.size(); ++i) {
            pe.primitive_inlier_points(inliers[i], extracted[i]);
        }

        graph_extractor ge(extracted, inliers, 0.1);
        std::string graphdir = "/home/nbore/Workspace/mapping_library/graph_primitives/graphs/";
        std::stringstream ss;
        ss << "graph" << std::setfill('0') << std::setw(6) << counter << ".dot";
        std::cout << ss.str() << std::endl;
        std::string graphfile = graphdir + ss.str();
        std::string imagefile = graphdir + "test.png";
        ge.generate_dot_file(graphfile);
        std::string command = "dot -Tpng " + graphfile + " > " + imagefile + " && gvfs-open " + imagefile;
        system(command.c_str());

        ss.str("");
        ss << "indices" << std::setfill('0') << std::setw(6) << counter << ".txt";
        std::string indexfile = graphdir + ss.str();
        ge.generate_index_file(indexfile);

        ++counter;
    }

    viewer.join_thread();

    return 0;
}

