#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <depthimage_to_laserscan/DepthImageToLaserScanROS.h>
#include <depthimage_to_laserscan/DepthImageToLaserScan.h>

depthimage_to_laserscan::DepthImageToLaserScan d;

sensor_msgs::LaserScan::ConstPtr msg_laser;
sensor_msgs::CameraInfo msg_camera;
sensor_msgs::ImageConstPtr msg_depth;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  msg_laser = msg;

  ROS_INFO("Depth Data Encoding: %s", msg_depth->encoding.c_str());

    // Display the LaserScan data
  ROS_INFO("Received LaserScan:");
  ROS_INFO("Header:");
  ROS_INFO("  Seq: %d", msg_laser->header.seq);
  ROS_INFO("  Stamp: %f", msg_laser->header.stamp.toSec());
  ROS_INFO("  Frame ID: %s", msg_laser->header.frame_id.c_str());
  ROS_INFO("Angle Min: %f", msg_laser->angle_min);
  ROS_INFO("Angle Max: %f", msg_laser->angle_max);
  ROS_INFO("Angle Increment: %f", msg_laser->angle_increment);
  ROS_INFO("Time Increment: %f", msg_laser->time_increment);
  ROS_INFO("Scan Time: %f", msg_laser->scan_time);
  ROS_INFO("Range Min: %f", msg_laser->range_min);
  ROS_INFO("Range Max: %f", msg_laser->range_max);
  ROS_INFO("Ranges:");
  ROS_INFO("Ranges Size: %d",msg_laser->ranges.size());
  int length_range = msg_laser->ranges.size();
  for (size_t i = 0.45 * length_range; i < 0.55 * length_range; ++i) {
    ROS_INFO("  [%zu]: %f", i, msg_laser->ranges[i]);
  }
  ROS_INFO("Intensities:");
  ROS_INFO("Intensities Size: %d",msg_laser->intensities.size());
  for (size_t i = 0; i < msg_laser->intensities.size(); ++i) {
    //ROS_INFO("  [%zu]: %f", i, msg_laser->intensities[i]);
  }
}

void cameraCallback(const sensor_msgs::CameraInfo& msg)
{
  msg_camera = msg;
  ROS_INFO("Width img: %d",msg_camera.width);
} 

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (msg->encoding == sensor_msgs::image_encodings::MONO16 || msg->encoding == "16UC1")
  {
    ROS_INFO("Depth Data Encoding: %s", msg->encoding.c_str());
    msg_depth = msg;
  }
  else
  {
    ROS_ERROR("Unsupported Depth Data Encoding: %s", msg->encoding.c_str());
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_subscriber");
  ros::NodeHandle nh;

  // Subscribe to the /scan topic
  ros::Subscriber sub = nh.subscribe("/scan", 10, scanCallback);
  ros::Subscriber sub_camera = nh.subscribe("/aditof_roscpp/camera_info", 10, cameraCallback);
  ros::Subscriber sub_depth= nh.subscribe("/aditof_roscpp/aditof_depth", 10, depthCallback);

  while(ros::ok())
  {
  // Spin and process incoming messages
  ros::spinOnce();

  }


  return 0;
}