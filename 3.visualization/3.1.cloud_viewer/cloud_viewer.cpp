#include <iostream>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int user_data;

void viewer_one_off(pcl::visualization::PCLVisualizer &viewer)
{
    viewer.setBackgroundColor(1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere(o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
}

void viewer_psycho(pcl::visualization::PCLVisualizer &viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape("text", 0);
    viewer.addText(ss.str(), 200, 300, "text", 0);
    // FIXME: possible race condition here:
    user_data++;
}

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("my_point_cloud.pcd", *cloud);
    pcl::visualization::CloudViewer viewer("cloud viewer");
    // blocks until the cloud is actually rendered
    viewer.showCloud(cloud);
    // use the following functions to get access to the underlying more advanced/powerful
    // This will only get called onc

    viewer.runOnVisualizationThreadOnce(viewer_one_off);
    // This will get called once per visualization iteration
    viewer.runOnVisualizationThread(viewer_psycho);
    while (!viewer.wasStopped())
    {
        user_data++;
    }
}