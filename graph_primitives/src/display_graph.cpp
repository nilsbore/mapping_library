#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

std::string screenshotfile;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
    pcl::visualization::PCLVisualizer* viewer = static_cast<pcl::visualization::PCLVisualizer*>(viewer_void);
    if (event.getKeySym () == "p" && event.keyDown ())
    {
        std::cout << "p was pressed => saving screenshot" << std::endl;
        viewer->saveScreenshot(screenshotfile);
    }
}

int main(int argc, char** argv)
{
    if (argc < 5) {
        std::cout << "Please supply the PCD file and index file you want to display..."  << argc << std::endl;
        return 0;
    }

    std::string pcdfile(argv[1]);
    std::string indexfile(argv[2]);
    std::string ordering(argv[3]);
    screenshotfile = std::string(argv[4]);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdfile, *cloud) == -1)
    {
        std::cout << "Please supply a valid PCL rgb cloud, couldn't open " << pcdfile << std::endl;
        return 0;
    }

    std::ifstream indices;
    indices.open(indexfile);

    std::vector<std::vector<int> > res;
    std::string s;
    while (std::getline(indices, s)) {
        std::istringstream is(s.c_str());
        std::istream_iterator<int> start(is), end;
        res.push_back(std::vector<int>(start, end));
    }

    std::istringstream iis(ordering.c_str());
    std::istream_iterator<int> istart(iis), iend;
    std::vector<int> order(istart, iend);

    if (order.size() != res.size()) {
        std::cout << "The order must be of the same size as the number of primitives..." << std::endl;
        return 0;
    }

    int colormap[][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 0, 255}, {255, 255, 0}, {64, 224, 208}};

    // filter out points that are far from the camera and thus will contain too much noise
    double dist = 2.0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, dist);
    pass.filter(*filtered_cloud);

    int index;
    int o;
    for (int i = 0; i < res.size(); ++i) {
        o = order[i] - 1;
        if (o == -1) {
           continue;
        }
        for (int j = 0; j < res[i].size(); ++j) {
            index = res[i][j];
            filtered_cloud->points[index].r = colormap[o][0];
            filtered_cloud->points[index].g = colormap[o][1];
            filtered_cloud->points[index].b = colormap[o][2];
        }
    }

    pcl::visualization::PCLVisualizer viewer;
    viewer.setBackgroundColor(0, 0, 0);
    viewer.initCameraParameters();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(filtered_cloud);
    viewer.addPointCloud<pcl::PointXYZRGB>(filtered_cloud, rgb, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             1, "cloud");
    viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }
    viewer.close();

    return 0;
}
