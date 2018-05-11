#include <string>
#include <fstream>
#include <iostream>
#include <ctime>
#include <sstream>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include "ros/ros.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>

class ImageCloudDataExport {
public:
    ImageCloudDataExport();

    void Run();

private:

    ros::NodeHandle node_handle_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber image_sub_;

    int pointcloud_counter_, image_counter_;
    int leading_zeros;
    std::string path_pointcloud_str_, path_image_str_;

    void VelodyneLidarCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud);

    void ImageCallback(const sensor_msgs::Image &in_image_msg);
};

ImageCloudDataExport::ImageCloudDataExport() :
        node_handle_("~") {
    pointcloud_counter_ = 0;
    image_counter_ = 0;
}


void ImageCloudDataExport::ImageCallback(const sensor_msgs::Image &in_image_msg) {
    ROS_INFO("ImageCallback: Start");
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat current_image = cv_image->image;

    if (image_counter_ % 20 == 0) {
        std::string string_counter = std::to_string(image_counter_);
        std::string file_path = path_image_str_ + "image_" + std::string(leading_zeros - string_counter.length(), '0') +
                                string_counter + ".jpg";

        cv::imwrite(file_path, current_image);
    }

    image_counter_++;
}

void ImageCloudDataExport::VelodyneLidarCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*in_sensor_cloud, *velodyne_cloud_ptr);

    std::string string_counter = std::to_string(pointcloud_counter_);
    std::string file_path =
            path_pointcloud_str_ + "scan_" + std::string(leading_zeros - string_counter.length(), '0') + string_counter;

    std::ofstream pointcloud_file(file_path + ".txt");
    if (!pointcloud_file.fail()) {
        for (size_t i = 0; i < velodyne_cloud_ptr->points.size(); i++) {
            pointcloud_file << velodyne_cloud_ptr->points[i].y << ", " << velodyne_cloud_ptr->points[i].z << ", "
                            << velodyne_cloud_ptr->points[i].x << ", " << velodyne_cloud_ptr->points[i].intensity
                            << std::endl;
        }
        pointcloud_file.close();
    }
    //save PCD file as well
    pcl::io::savePCDFileASCII(file_path + ".pcd", *velodyne_cloud_ptr);

    pointcloud_counter_++;
}


void ImageCloudDataExport::Run() {
    ros::NodeHandle private_node_handle("~");

    std::string image_src, points_src;
    private_node_handle.param<std::string>("image_src", image_src, "/image_raw");
    ROS_INFO("[ImageCloudDataExport] image_src: %s", image_src.c_str());

    private_node_handle.param<std::string>("points_src", points_src, "/points_raw");
    ROS_INFO("[ImageCloudDataExport] points_src: %s", points_src.c_str());
    cloud_sub_ = node_handle_.subscribe(points_src, 10, &ImageCloudDataExport::VelodyneLidarCallback, this);
    image_sub_ = node_handle_.subscribe(image_src, 10, &ImageCloudDataExport::ImageCallback, this);

    time_t rawtime;
    struct tm *timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", timeinfo);
    std::string datetime_str(buffer);

    const char *homedir;

    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }

    path_pointcloud_str_ = std::string(homedir) + "/output_" + datetime_str + "/pointcloud/";
    path_image_str_ = std::string(homedir) + "/output_" + datetime_str + "/image/";

    leading_zeros = 6;

    boost::filesystem::path path_pointcloud(path_pointcloud_str_.c_str());
    boost::filesystem::path path_image(path_image_str_.c_str());

    boost::filesystem::create_directories(path_image);
    boost::filesystem::create_directories(path_pointcloud);

    ROS_INFO("ImageCloudDataExport: PointCloud data stored in %s", path_pointcloud.c_str());
    ROS_INFO("ImageCloudDataExport: Image data stored in %s", path_image_str_.c_str());

    ROS_INFO("ImageCloudDataExport: Waiting for data...");
    ros::spin();

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_pointcloud_exporter");

    ImageCloudDataExport node;

    node.Run();

    return 0;
}
