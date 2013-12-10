#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include "primitive_extractor.h"
#include "primitive_visualizer.h"
#include "plane_primitive.h"
#include "sphere_primitive.h"
#include "cylinder_primitive.h"

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    //std::string filename = "/home/nbore/Downloads/home_data_ascii/scene11_ascii.pcd";
    //std::string filename = "/home/nbore/Data/rgbd_dataset_freiburg1_room/pointclouds/1305031910.765238.pcd";
    //std::string filename = "/home/nbore/Data/rgbd_dataset_freiburg1_room/pointclouds/1305031943.570021.pcd";
    //std::string filename = "/home/nbore/Data/table_scene_mug_stereo_textured.pcd";
    std::string filename = "/home/nbore/catkin_ws/myclouds2/cloud33.pcd";
    //std::string filename = "/home/nbore/Data/Primitives/cloud28.pcd";
    //std::string filename = "/home/nbore/catkin_ws/farclouds2/cloud100.pcd";
    //std::string filename = "/home/nbore/catkin_ws/xtionclouds/cloud1620.pcd";
    //std::string filename = "/home/nbore/Data/SemanticMap/20131206/patrol_run_1/room_0/complete_cloud.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *cloud) == -1)
    {
        std::cout << "Couldn't read file " << filename << std::endl;
        return 0;
    }

    std::vector<base_primitive*> primitives = { new plane_primitive(), new sphere_primitive(), new cylinder_primitive() };
    primitive_params params;
    /*params.octree_res = 0.2;
    params.normal_neigbourhood = 0.02;
    params.inlier_threshold = 0.02;
    params.angle_threshold = 0.4;
    params.add_threshold = 0.01;
    params.min_shape = 1000;
    params.inlier_min = params.min_shape;
    params.connectedness_res = 0.01;*/

    primitive_visualizer viewer;
    primitive_extractor extractor(cloud, primitives, params, &viewer);
    viewer.cloud = extractor.get_cloud();
    viewer.cloud_changed = true;
    viewer.cloud_normals = extractor.get_normals();
    viewer.normals_changed = true;
    viewer.create_thread();
    std::vector<base_primitive*> extracted;
    extractor.extract(extracted);
    viewer.join_thread();

    return 0;
}
