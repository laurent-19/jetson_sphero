#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <depthimage_to_laserscan/DepthImageToLaserScan.h>

class DepthToLaserScanNode
{
public:
  DepthToLaserScanNode()
  {

    ros::NodeHandle nh;

    // Subscribe to the depth image topic
    image_sub_ = nh.subscribe("/aditof_roscpp/aditof_depth", 1, &DepthToLaserScanNode::depthImageCallback, this);

    // Subscribe to the camera info topic
    info_sub_ = nh.subscribe("/aditof_roscpp/aditof_camera_info", 1, &DepthToLaserScanNode::cameraInfoCallback, this);

    // Advertise the laser scan topic
    scan_pub_ = nh.advertise<sensor_msgs::LaserScan>("/depth_line_extractor", 1);

    // Initialize the depthimage_to_laserscan converter
    converter_.reset(new depthimage_to_laserscan::DepthImageToLaserScan());
  }

  void depthImageCallback(const sensor_msgs::Image::ConstPtr& depth_msg)
  {
    if (!camera_info_received_)
    {
      ROS_INFO_ONCE("Camera info not received yet. Skipping depth image conversion.");
      return;
    }

    // Convert the depth image to laser scan using the provided implementation
    sensor_msgs::LaserScanPtr laser_scan_msg = converter_->convert_msg(depth_msg, camera_info_msg_);

    // Print the laser scan data
    ROS_INFO_STREAM("Laser Scan Data:");
    ROS_INFO_STREAM("Header: " << laser_scan_msg->header);
    ROS_INFO_STREAM("Angle Min: " << laser_scan_msg->angle_min);
    ROS_INFO_STREAM("Angle Max: " << laser_scan_msg->angle_max);
    ROS_INFO_STREAM("Angle Increment: " << laser_scan_msg->angle_increment);
    ROS_INFO_STREAM("Time Increment: " << laser_scan_msg->time_increment);
    ROS_INFO_STREAM("Scan Time: " << laser_scan_msg->scan_time);
    ROS_INFO_STREAM("Range Min: " << laser_scan_msg->range_min);
    ROS_INFO_STREAM("Range Max: " << laser_scan_msg->range_max);
    ROS_INFO_STREAM("Ranges: " << laser_scan_msg->ranges.size());
    ROS_INFO_STREAM("Ranges: " << laser_scan_msg->ranges[laser_scan_msg->ranges.size()/2]);

    // Publish the laser scan
    scan_pub_.publish(laser_scan_msg);
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    // Store the camera info for later use in depth image conversion
    camera_info_msg_ = info_msg;
    camera_info_received_ = true;
  }

private:
  ros::Subscriber image_sub_;
  ros::Subscriber info_sub_;
  ros::Publisher scan_pub_;
  boost::shared_ptr<depthimage_to_laserscan::DepthImageToLaserScan> converter_;
  sensor_msgs::CameraInfo::ConstPtr camera_info_msg_;
  bool camera_info_received_ = false;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_to_laserscan_node");
  
  DepthToLaserScanNode node();

  ros::spin();

  return 0;
}
