#include <iostream>
#include <pcl/common/io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    // load point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    int ret = pcl::io::loadPCDFile("table_scene_mug_stereo_textured.pcd", *cloud);
    if (ret < 0)
    {
        std::cerr << "Failed to load PCD file!" << std::endl;
        return -1;
    }
    std::cout << "Loaded point cloud: " << cloud->width << " x " << cloud->height << std::endl;
    std::cout << "size: " << cloud->points.size() << std::endl;
    std::cout << "is_dense: " << cloud->is_dense << std::endl;
    if (cloud->height == 1)
    {
        std::cerr << "Warning: Point cloud is unorganized (height == 1)." << std::endl;
    }
    else
    {
        std::cout << "Point cloud is organized (height > 1)." << std::endl;
    }
    // estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);
    std::cout << "完成: " << std::endl;
    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals);

    while (!viewer.wasStopped())
    {
        viewer.spin();
    }
    return 0;
}