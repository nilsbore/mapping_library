#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <boost/algorithm/string.hpp>

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
    std::string indexfiles(argv[2]);
    std::string orderings(argv[3]);
    screenshotfile = std::string(argv[4]);
    
    std::cout << "Arguments: " << pcdfile << "\n" << indexfiles << "\n" << orderings << "\n" << screenshotfile << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcdfile, *cloud) == -1)
    {
        std::cout << "Please supply a valid PCL rgb cloud, couldn't open " << pcdfile << std::endl;
        return 0;
    }
    
    std::vector<std::string> orderings_v;
    boost::split(orderings_v, orderings, boost::is_any_of(","));
    std::cout << "Orderings length: " << orderings_v.size() << std::endl;
    
    /*for (std::string& ord : orderings_v) {
        std::cout << ord << std::endl;
    }*/
    
    std::vector<std::string> indices_v;
    boost::split(indices_v, indexfiles, boost::is_any_of(" \t"));
    std::cout << "Indices length: " << indices_v.size() << std::endl;
    
    /*for (std::string& ord : indices_v) {
        std::cout << ord << std::endl;
    }*/
    
    if (indices_v.size() != orderings_v.size()) {
        std::cout << "Primitive indices and orderings must be the same size!" << std::endl;
    }
    
    int colormap[][3] = {{0, 255, 0}, {255, 255, 0}, {0, 0, 255}, {255, 0, 0}, {255, 0, 255}, {64, 224, 208}};
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
    /*int colormap[][3] = {{166,206,227},
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
                         {177,89,40}};*/
                         
    for (int j = 0; j < cloud->points.size(); ++j) {
        cloud->points[j].r = 80;
        cloud->points[j].g = 80;
        cloud->points[j].b = 80;
    }
    
    pcl::visualization::PCLVisualizer viewer;
    viewer.setBackgroundColor(0, 0, 0);
    viewer.initCameraParameters();
    
    for (size_t m = 0; m < indices_v.size(); ++m) {
        pcl::PointXYZRGB point;
        point.x = point.y = point.z = 0;
        point.r = 255.0; point.g = 0.0; point.b = 0.0;
        float number_points = 0.0f;
        std::string indexfile = indices_v[m];
        std::string ordering = orderings_v[m];

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

        int index;
        int o;
        std::cout << "Display nodes: " << order.size() << std::endl;
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > points;
        for (int i = 0; i < order.size(); ++i) {
            o = order[i] - 1;
            std::cout << order[i] << std::endl;
            if (o == -1) {
               continue;
            }
            points.push_back(Eigen::Vector3f(0, 0, 0));
            float number_pointsi = 0;
            for (int j = 0; j < res[i].size(); ++j) {
                index = res[i][j];
                cloud->points[index].r = colormap[o%12][0];
                cloud->points[index].g = colormap[o%12][1];
                cloud->points[index].b = colormap[o%12][2];
                point.getVector3fMap() += cloud->points[index].getVector3fMap();
                points.back() += cloud->points[index].getVector3fMap();
                number_points += 1.0f;
                number_pointsi += 1.0f;
            }
            points.back() *= 1.0f/number_pointsi;
        }
        std::sort(points.begin(), points.end(), [](const Eigen::Vector3f& first, const Eigen::Vector3f& second) { return first(0) < second(0); });
        point.x = points[int(points.size()/2)](0);
        std::sort(points.begin(), points.end(), [](const Eigen::Vector3f& first, const Eigen::Vector3f& second) { return first(1) < second(1); });
        point.y = points[int(points.size()/2)](1);
        point.z = 3.0f;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr single_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        //point.getVector3fMap() *= 1.0f/number_points;
        //point.getVector3fMap()(2) += 2.0f;
        single_cloud->points.push_back(point);
        /*pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> single_rgb(single_cloud);
        viewer.addPointCloud<pcl::PointXYZRGB>(single_cloud, single_rgb, std::string("point") + std::to_string(m));
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                 30, std::string("point") + std::to_string(m));*/
    }

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");
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
