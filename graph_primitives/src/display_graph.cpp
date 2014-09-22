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

    std::cout << "Ordering: " << ordering << std::endl;
    std::istringstream iis(ordering.c_str());
    std::istream_iterator<int> istart(iis), iend;
    std::vector<int> order(istart, iend);

    if (order.size() != res.size()) {
        std::cout << "The order must be of the same size as the number of primitives..." << std::endl;
        //return 0;
    }

    //int colormap[][3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}, {255, 0, 255}, {255, 255, 0}, {64, 224, 208}};
    /*int colormap[][3] = {{141,211,199},
                         {255,255,179},
                         {190,186,218},
                         {251,128,114},
                         {128,177,211},
                         {253,180,98},
                         {179,222,105},
                         {252,205,229},
                         {217,217,217},
                         {188,128,189},
                         {204,235,197},
                         {255,237,111}};*/
    int colormap[][3] = {{166,206,227},
                         {31,120,180},
                         {178,223,138},
                         {51,160,44},
                         {251,154,153},
                         {227,26,28},
                         {253,191,111},
                         {255,127,0},
                         {202,178,214},
                         {106,61,154},
                         {255,255,153},
                         {177,89,40}};

    // filter out points that are far from the camera and thus will contain too much noise
    double dist = 2.0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*cloud, *filtered_cloud);
    /*pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, dist);
    pass.filter(*filtered_cloud);*/
    

    for (int j = 0; j < filtered_cloud->points.size(); ++j) {
        filtered_cloud->points[j].r = 80;
        filtered_cloud->points[j].g = 80;
        filtered_cloud->points[j].b = 80;
    }
    int index;
    int o;
    std::cout << "Display nodes: " << order.size() << std::endl;
    for (int i = 0; i < order.size(); ++i) {
        o = order[i] - 1;
        std::cout << order[i] << std::endl;
        if (o == -1) {
           continue;
        }
        for (int j = 0; j < res[i].size(); ++j) {
            index = res[i][j];
            filtered_cloud->points[index].r = colormap[o%6][0];
            filtered_cloud->points[index].g = colormap[o%6][1];
            filtered_cloud->points[index].b = colormap[o%6][2];
        }
    }

    pcl::visualization::PCLVisualizer viewer;
    viewer.setBackgroundColor(0, 0, 0);
    viewer.initCameraParameters();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(filtered_cloud);
    viewer.addPointCloud<pcl::PointXYZRGB>(filtered_cloud, rgb, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             5, "cloud");
    viewer.registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }
    viewer.close();

    return 0;
}
