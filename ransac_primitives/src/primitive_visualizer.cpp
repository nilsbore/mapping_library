#include "primitive_visualizer.h"

void primitive_visualizer::run_visualizer()
{
    // Wait until visualizer window is closed.
    while (!viewer->wasStopped())
    {
        lock();
        if (cloud_changed) {
            viewer->removePointCloud("cloud");
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
            viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                     1, "cloud");
            cloud_changed = false;
        }
        if (normals_changed) {
            viewer->removePointCloud("normals");
            viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, cloud_normals, 100, 1e-2f, "normals");
            normals_changed = false;
        }
        viewer->spinOnce(100);
        unlock();
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    viewer->close();
}

void primitive_visualizer::lock()
{
    pthread_mutex_lock(&mutex);
}

void primitive_visualizer::unlock()
{
    pthread_mutex_unlock(&mutex);
}

void* viewer_thread(void* ptr)
{
    ((primitive_visualizer*)ptr)->run_visualizer();
    pthread_exit(NULL);
}

void primitive_visualizer::create_thread()
{
    pthread_create(&my_viewer_thread, NULL, viewer_thread, this);
}

void primitive_visualizer::join_thread()
{
    pthread_join(my_viewer_thread, NULL);
}
