#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "primitive_extractor.h"
#include "primitive_visualizer.h"
#include "plane_primitive.h"

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    std::string filename = "/home/nbore/Downloads/home_data_ascii/scene11_ascii.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *cloud) == -1)
    {
        std::cout << "Couldn't read file " << filename << std::endl;
        return 0;
    }

    std::vector<base_primitive*> primitives = { new plane_primitive() };
    primitive_extractor extractor(cloud, primitives);
    primitive_visualizer viewer;
    viewer.cloud = cloud;
    viewer.cloud_normals = extractor.get_normals();
    viewer.run_visualizer();

    return 0;
}
