#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// 使用VoxelGrid对点云下采样进行稀疏处理
int main()
{
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
    // Fill in the cloud data
    pcl::PCDReader reader;
    // load pcd file
    reader.read("table_scene_lms400.pcd", *cloud);
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points ("
              << pcl::getFieldsList(*cloud) << ")." << std::endl;
    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    // 越大得到的点云越稀疏
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    // sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points ("
              << pcl::getFieldsList(*cloud_filtered) << ")." << std::endl;
    pcl::PCDWriter writer;
    // writer point cloud to pcd file
    writer.write("table_scene_lms400_downsampled.pcd", *cloud_filtered, Eigen::Vector4f::Zero(),
                 Eigen::Quaternionf::Identity(), false);
    return 0;
}