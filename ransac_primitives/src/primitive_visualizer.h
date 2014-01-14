#ifndef PRIMITIVE_VISUALIZER_H
#define PRIMITIVE_VISUALIZER_H

#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

class primitive_visualizer
{
private:
    pthread_mutex_t mutex;
    pthread_t my_viewer_thread;
public:
    bool cloud_changed;
    bool normals_changed;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;
    pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals;
    void create_thread();
    void join_thread();
    void lock();
    void unlock();
    void run_visualizer();
    primitive_visualizer() : viewer(new pcl::visualization::PCLVisualizer("3D Viewer"))
    {
        viewer->setBackgroundColor(0, 0, 0);
        // Starting visualizer
        //viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();
        cloud_changed = false;
        normals_changed = false;
        if (pthread_mutex_init(&mutex, NULL) != 0) {
            std::cout << "mutex init failed" << std::endl;
        }
    }
};

#endif // PRIMITIVE_VISUALIZER_H
