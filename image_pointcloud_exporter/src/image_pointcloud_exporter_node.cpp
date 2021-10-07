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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <opencv2/highgui/highgui.hpp>

#include <boost/filesystem.hpp>

class ImageCloudDataExport
{
public:
  ImageCloudDataExport();

  void Run();

private:

  ros::NodeHandle node_handle_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber image_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sync_sub_;
  message_filters::Subscriber<sensor_msgs::Image> *image_sync_sub_;

  typedef
  message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> SyncPolicyT;

  message_filters::Synchronizer<SyncPolicyT>
    *data_synchronizer_;

  size_t image_frame_counter_, cloud_frame_counter_;
  bool sync_topics_;
  int leading_zeros;
  std::string path_pointcloud_str_, path_image_str_;

  void LidarCloudCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud);

  void ImageCallback(const sensor_msgs::ImageConstPtr &in_image_msg);

  void SyncedDataCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud,
    const sensor_msgs::ImageConstPtr &in_image_msg);
};

ImageCloudDataExport::ImageCloudDataExport() :
  node_handle_()
{
  image_frame_counter_ = 0;
  cloud_frame_counter_ = 0;
}


void ImageCloudDataExport::ImageCallback(const sensor_msgs::ImageConstPtr &in_image_msg)
{
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat current_image = cv_image->image;

  std::string string_counter = std::to_string(image_frame_counter_);
  std::string file_path = path_image_str_ + "image_" + std::string(leading_zeros - string_counter.length(), '0') +
                          string_counter + ".jpg";

  cv::imwrite(file_path, current_image);
  if(!sync_topics_)
  {
    image_frame_counter_++;
  }

}

void
ImageCloudDataExport::SyncedDataCallback(
  const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud,
  const sensor_msgs::ImageConstPtr &in_image_msg)
{
  ROS_INFO("[ImageCloudDataExport] Frame Synced: %d", in_image_msg->header.stamp.sec);
  LidarCloudCallback(in_sensor_cloud);
  ImageCallback(in_image_msg);
  cloud_frame_counter_++;
  image_frame_counter_++;
}

void ImageCloudDataExport::LidarCloudCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*in_sensor_cloud, *cloud_ptr);

  std::string string_counter = std::to_string(cloud_frame_counter_);
  std::string file_path =
    path_pointcloud_str_ + "scan_" + std::string(leading_zeros - string_counter.length(), '0') + string_counter;

  /*std::ofstream pointcloud_file(file_path + ".txt");
  if (!pointcloud_file.fail())
  {
    pointcloud_file << cloud_ptr->points.size() << std::endl;
    for (size_t i = 0; i < cloud_ptr->points.size(); i++)
    {
      pointcloud_file << cloud_ptr->points[i].y << " " << cloud_ptr->points[i].z << " "
                      << cloud_ptr->points[i].x << " " << int(cloud_ptr->points[i].intensity)
                      << std::endl;
    }
    pointcloud_file.close();
  }*/
  //save PCD file as well
  pcl::io::savePCDFileBinaryCompressed(file_path + ".pcd", *cloud_ptr);
  if(!sync_topics_)
  {
    cloud_frame_counter_++;
  }
}


void ImageCloudDataExport::Run()
{
  ros::NodeHandle private_node_handle("~");

  std::string image_src, points_src;
  private_node_handle.param<std::string>("image_src", image_src, "image_raw");
  ROS_INFO("[ImageCloudDataExport] image_src: %s", image_src.c_str());

  private_node_handle.param<std::string>("points_src", points_src, "points_raw");
  ROS_INFO("[ImageCloudDataExport] points_src: %s", points_src.c_str());

  private_node_handle.param<bool>("sync_topics", sync_topics_, false);
  ROS_INFO("[ImageCloudDataExport] sync_topics: %d", sync_topics_);

  if(!sync_topics_)
  {
    cloud_sub_ = node_handle_.subscribe(points_src, 10, &ImageCloudDataExport::LidarCloudCallback, this);
    image_sub_ = node_handle_.subscribe(image_src, 10, &ImageCloudDataExport::ImageCallback, this);
  }
  else
  {
    cloud_sync_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
                                                                           points_src,
                                                                           1);
    image_sync_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(node_handle_,
                                                                     image_src,
                                                                     1);
    data_synchronizer_ =
        new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(10),
                                                       *cloud_sync_sub_,
                                                       *image_sync_sub_);
    data_synchronizer_->registerCallback(
        boost::bind(&ImageCloudDataExport::SyncedDataCallback, this, _1, _2));
  }

  time_t rawtime;
  struct tm *timeinfo;
  char buffer[80];

  time(&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", timeinfo);
  std::string datetime_str(buffer);

  const char *homedir;

  if ((homedir = getenv("HOME")) == NULL)
  {
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_pointcloud_exporter");

  ImageCloudDataExport node;

  node.Run();

  return 0;
}
