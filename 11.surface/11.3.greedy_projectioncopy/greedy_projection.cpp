#include <pcl/common/io.h> // for copyPointCloud
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h> // for KdTree
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_subdivision.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkPolyDataPointSampler.h>
#include <vtkSmartPointer.h>
int main(int argc, char **argv)
{

    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile("./data/cloud_dipan.pcd", cloud_blob);
    pcl::fromPCLPointCloud2(cloud_blob, *cloud);
    //* the data should be available in cloud
    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);
    // Initialize objects

    pcl::PolygonMesh triangles;
#if 0
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(0.025);

    // Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

#else
    pcl::Poisson<pcl::PointNormal> poisson;

    // 设置参数
    poisson.setDepth(11); // 八叉树深度
    poisson.setInputCloud(cloud_with_normals);
    // poisson.setPointWeight(4.0f);    // 点云权重
    // poisson.setScale(1.1f);          // 尺度因子
    // poisson.setSolverDivide(8);      // 求解器划分
    // poisson.setIsoDivide(8);         // 等值面划分
    // poisson.setManifold(true); // 保持流形
    // poisson.setOutputPolygons(true); // 输出多边形
    // 执行重建
    poisson.reconstruct(triangles);

#endif

#if 0
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 1. 将PCL PolygonMesh转换为VTK PolyData
    vtkSmartPointer<vtkPolyData> vtk_mesh = vtkSmartPointer<vtkPolyData>::New();
    pcl::io::mesh2vtk(triangles, vtk_mesh);

    // 2. 创建点采样器
    vtkSmartPointer<vtkPolyDataPointSampler> sampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
    sampler->SetInputData(vtk_mesh);
    // sampler->SetDistance(0.0); // 设置为0，通过SetNumberOfPoints控制采样数量
    // 4. 执行采样
    sampler->Update();

    // 5. 获取采样结果
    vtkPolyData *sampled = sampler->GetOutput();

    // 6. 转换为PCL点云
    sampled_cloud->resize(sampled->GetNumberOfPoints());
    for (vtkIdType i = 0; i < sampled->GetNumberOfPoints(); ++i)
    {
        double *point = sampled->GetPoint(i);
        (*sampled_cloud)[i].x = point[0];
        (*sampled_cloud)[i].y = point[1];
        (*sampled_cloud)[i].z = point[2];
    }

    pcl::visualization::PCLVisualizer visu("Alignment");
    if (pcl::console::find_argument(argc, argv, "-s") >= 0)
    {
        visu.addPointCloud<pcl::PointXYZ>(
            cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, 0, 255, 0), "cloud");
    }
    else
    {
        visu.addPointCloud<pcl::PointXYZ>(
            sampled_cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(sampled_cloud, 255, 0, 0),
            "sampled_cloud");
    }

    visu.spin();
#else
    pcl::visualization::PCLVisualizer visu("Alignment");
    visu.addPolygonMesh(triangles, "mesh");
    // 设置网格显示属性
    visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.8, 0.8, "mesh");
    visu.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "mesh");

    std::stringstream ss;
    ss << "顶点数: " << triangles.cloud.width * triangles.cloud.height << ", 面数: " << triangles.polygons.size();
    visu.addText(ss.str(), 10, 10, 12, 1.0, 1.0, 1.0, "mesh_info");

    visu.initCameraParameters();
    visu.spin();
#endif
    // Finish
    return 0;
}