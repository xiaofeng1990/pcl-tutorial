#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>

// 使用PassThrough对选定轴进行区域范围过滤
int main()
{
    std::cout << "pass through filter" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    cloud->width = 5;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (auto &point : *cloud)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
    std::cerr << "Cloud before filtering: " << std::endl;
    for (const auto &point : *cloud)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    // 过滤z轴，可以选x，y，z
    pass.setFilterFieldName("z");
    // 对过滤范围取反
    //  pass.setNegative(true);
    // 设置过滤范围
    pass.setFilterLimits(0.0, 1.0);
    // 输出过滤结果
    pass.filter(*cloud_filtered);
    std::cerr << "Cloud after filtering: " << std::endl;
    for (const auto &point : *cloud_filtered)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;
    return 0;
}