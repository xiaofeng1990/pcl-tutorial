#include <pcl/ModelCoefficients.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/pcl_visualizer.h>

// 检测点云的多边形轮廓
int main(int argc, char **argv)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;

    reader.read("./data/table_scene_mug_stereo_textured.pcd", *cloud);
    // Build a filter to remove spurious NaNs and scene background
    // 根据z轴过滤
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1.1);
    pass.filter(*cloud_filtered);
    std::cerr << "PointCloud after filtering has: " << cloud_filtered->size() << " data points." << std::endl;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    std::cerr << "PointCloud after segmentation has: " << inliers->indices.size() << " inliers." << std::endl;

    // Project the model inliers
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    // proj.setIndices (inliers);
    proj.setInputCloud(cloud_filtered);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);
    std::cerr << "PointCloud after projection has: " << cloud_projected->size() << " data points." << std::endl;

    // Create a Concave Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud_projected);
    chull.setAlpha(1.0);
    chull.reconstruct(*cloud_hull);

    std::cerr << "Concave hull has: " << cloud_hull->size() << " data points." << std::endl;

    pcl::PCDWriter writer;
    writer.write("table_scene_mug_stereo_textured_hull.pcd", *cloud_hull, false);

    pcl::visualization::PCLVisualizer visu("Alignment");

    visu.addPointCloud<pcl::PointXYZ>(
        cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 0, 255, 0), "cloud");
    visu.addPointCloud<pcl::PointXYZ>(
        cloud_hull, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_hull, 255, 0, 0),
        "cloud_hull");

    visu.spin();
    return 0;
}