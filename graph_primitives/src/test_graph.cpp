#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <dirent.h>

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
    std::string dirname = "/home/nbore/catkin_ws/xtionclouds";
    //std::string dirname = "/home/nbore/Data/pcldata/Pcd";
    std::vector<std::string> files;
    if (!convenience::read_directory(files, dirname))
    {
        std::cout << "Directory empty..." << std::endl;
        return 0;
    }

    std::vector<base_primitive*> primitives = { new plane_primitive(), new cylinder_primitive() };
    primitive_params params;
    params.octree_res = 0.2;
    params.normal_neigbourhood = 0.03;
    params.inlier_threshold = 0.02;
    params.angle_threshold = 0.3;
    params.add_threshold = 0.01;
    params.min_shape = 1000;
    params.inlier_min = params.min_shape;
    params.connectedness_res = 0.01;

    primitive_visualizer viewer;
    viewer.create_thread();

    for (const std::string& file : files) {

        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (file, *cloud) == -1)
        {
            std::cout << "Couldn't read file " << file << std::endl;
            return 0;
        }

        std::cout << "Processing " << file << ".." << std::endl;

        primitive_extractor pe(cloud, primitives, params, &viewer);
        viewer.cloud = pe.get_cloud();
        viewer.cloud_changed = true;
        viewer.cloud_normals = pe.get_normals();
        viewer.normals_changed = true;
        std::vector<base_primitive*> extracted;
        pe.extract(extracted);

        std::vector<Eigen::MatrixXd> inliers;
        inliers.resize(extracted.size());
        for (int i = 0; i < extracted.size(); ++i) {
            pe.primitive_inlier_points(inliers[i], extracted[i]);
        }

        graph_extractor ge(extracted, inliers, 0.1);
        std::string graphfile = "/home/nbore/Workspace/mapping_library/graph_primitives/graphs/test3.dot";
        ge.generate_dot_file(graphfile);

        break;
    }

    viewer.join_thread();



    return 0;
}

