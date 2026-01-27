#include <pcl/common/io.h> // for copyPointCloud
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
int main(int argc, char **argv)
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Load bun0.pcd -- should be available with the PCL archive in test
    pcl::io::loadPCDFile("./data/cloud_dipan.pcd", *cloud);

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    // pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(1);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);
    // mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::SAMPLE_LOCAL_PLANE);
    // mls.setUpsamplingRadius(0.01);    // 局部平面半径
    // mls.setUpsamplingStepSize(0.005); // 局部平面步长
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::VOXEL_GRID_DILATION);
    mls.setDilationVoxelSize(0.005); // 体素大小
    mls.setDilationIterations(2);    // 膨胀次数
    // Reconstruct
    mls.process(*mls_points);

    // Save output
    pcl::io::savePCDFile("bun0-mls.pcd", *mls_points);

    pcl::visualization::PCLVisualizer visu("Alignment");
    // visu.addPointCloudNormals<pcl::PointXYZ, pcl::PointNormal>(cloud, mls_points, 1, 0.01, "normals");
    // visu.addPointCloud<pcl::PointXYZ>(
    //     cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 255, 0, 0), "cloud");

    if (pcl::console::find_argument(argc, argv, "-s") >= 0)
    {
        visu.addPointCloud<pcl::PointXYZ>(
            cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 255, 0, 0), "cloud");
    }
    else
    {
        visu.addPointCloudNormals<pcl::PointNormal>(mls_points, 1, 0.01, "normals");
    }
    visu.spin();
    return 0;
}