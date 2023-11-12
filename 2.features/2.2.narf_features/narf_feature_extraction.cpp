#include <iostream>
#include <pcl/common/file_io.h>
#include <pcl/console/parse.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>

typedef pcl::PointXYZ PointType;

float angular_resolution = 0.5f;
float support_size = 0.2f;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
bool setUnseenToMaxRange = false;
bool rotation_invariant = true;

// help

void printUsage(const char *progName)
{
    std::cout << "\n\nUsage: " << progName << " [options] <scene.pcd>\n\n"
              << "Optjions: \n"
              << "-------------------------------------------\n"
              << "-r <float>   angular resolution in degrees (default " << angular_resolution << ")\n"
              << "-c <int>     coordinate frame (default " << (int)coordinate_frame << ")\n"
              << "-m           Treat all unseen points to max range\n"
              << "-s <float>   support size for the interest points (diameter of the used sphere - "
                 "default "
              << support_size << ")\n"
              << "-o <0/1>     switch rotational invariant version of the feature on/off"
              << " (default " << (int)rotation_invariant << ")\n"
              << "-h           this help\n"
              << "\n\n";
}

int main(int argc, char **argv)
{

    if (pcl::console::find_argument(argc, argv, "-h") >= 0)
    {
        printUsage(argv[0]);
        return 0;
    }

    if (pcl::console::find_argument(argc, argv, "-m") >= 0)
    {
        setUnseenToMaxRange = true;
        std::cout << "Setting unseen values in range image to maximum range readings.\n";
    }
    if (pcl::console::parse(argc, argv, "-o", rotation_invariant) >= 0)
    {
        std::cout << "Switching rotation invariant feature version " << (rotation_invariant ? "on" : "off") << ".\n";
    }
    int tmp_coordinate_frame;
    if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0)
    {
        coordinate_frame = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame);
        std::cout << "Using coordinate frame " << (int)coordinate_frame << ".\n";
    }
    if (pcl::console::parse(argc, argv, "-s", support_size) >= 0)
        std::cout << "Setting support size to " << support_size << ".\n";

    if (pcl::console::parse(argc, argv, "-r", angular_resolution) >= 0)
        std::cout << "Setting angular resolution to " << angular_resolution << "deg.\n";

    angular_resolution = pcl::deg2rad(angular_resolution);
    // ------------------------------------------------------------------
    // -----Read pcd file or create example point cloud if not given-----
    // ------------------------------------------------------------------

    pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType> &point_cloud = *point_cloud_ptr;

    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
    Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity());
    // 从命令行中获取pcd文件
    std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument(argc, argv, "pcd");
    if (!pcd_filename_indices.empty())
    {
        std::string filename = argv[pcd_filename_indices[0]];
        if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
        {
            std::cerr << "Was not able to open file \"" << filename << "\".\n";
            printUsage(argv[0]);
            return 0;
        }
        scene_sensor_pose =
            Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0], point_cloud.sensor_origin_[1],
                                                 point_cloud.sensor_origin_[2])) *
            Eigen::Affine3f(point_cloud.sensor_orientation_);
        std::string far_ranges_filename = pcl::getFilenameWithoutExtension(filename) + "_far_ranges.pcd";
        if (pcl::io::loadPCDFile(far_ranges_filename.c_str(), far_ranges) == -1)
            std::cout << "Far ranges file \"" << far_ranges_filename << "\" does not exists.\n";
    }
    else
    {
        setUnseenToMaxRange = true;
        std::cout << "\nNo *.pcd file given => Generating example point cloud.\n\n";
        setUnseenToMaxRange = true;
        std::cout << "\nNo *.pcd file given => Generating example point cloud.\n\n";
        for (float x = -0.5f; x <= 0.5f; x += 0.01f)
        {
            for (float y = -0.5f; y <= 0.5f; y += 0.01f)
            {
                PointType point;
                point.x = x;
                point.y = y;
                point.z = 2.0f - y;
                point_cloud.push_back(point);
            }
        }
        point_cloud.width = point_cloud.size();
        point_cloud.height = 1;
    }
    // -----------------------------------------------
    // -----Create RangeImage from the PointCloud-----
    // -----------------------------------------------

    return 0;
}