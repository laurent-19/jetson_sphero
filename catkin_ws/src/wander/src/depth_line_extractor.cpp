#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include </home/lauruslinux/projects/aditof_sdk/deps/opencv/modules/core/include/opencv2/core/mat.hpp>

#define SPEED 0.2

using namespace ros;

geometry_msgs::Twist command;

int distance  = 0;

void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  // Extract the depth image data from the message
  // Assuming the depth image is of type sensor_msgs/Image
  const std::vector<uint8_t>& depthData = msg->data;

  int imageWidth = msg->width;
  int imageHeight = msg->height;

  // Select the line to process (e.g., middle row)
  int lineIndex = imageHeight / 2;

  // Define the threshold value for obstacle detection
  int obstacleThreshold = 150;

  const std::string encoding = msg->encoding;

  ROS_INFO("Encoding %s:", encoding.c_str());

  // Print the extracted line data
  ROS_INFO("Line %d:", lineIndex);

  // Extract the specified line from the depth image
  std::vector<uint8_t> line_data;
  for (int i = 0; i < imageWidth; ++i) {
      uint8_t depth = depthData[lineIndex * imageWidth + i];
      line_data.push_back(depth);
  }

  // Convert sensor_msgs::Image to OpenCV Mat
  cv_bridge::CvImagePtr cvImage;
  try
  {
    cvImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGBA8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Access the depth image
  cv::Mat depthImage = cvImage->image;
  // Convert depthData to int16_t* data
  int dataSize = depthData.size() / sizeof(int16_t);
  //cv::Mat depthImage = cv::Mat(imageHeight,imageWidth,CV_16UC1,depthData.data());

  // DEclare central point
  cv::Point2d pointxy(lineIndex, imageWidth / 2);

  // Compute distance based on previous
  //distance = static_cast<int>(distance * 0.7 + depthImage.at<ushort>(pointxy) * 0.3);
  distance = static_cast<int>(depthImage.at<ushort>(pointxy));
  ROS_INFO("Dist %d:", distance);

  // Distances on the extracted line
    std::vector<int> line_distances;
    for (int i = 0; i < imageWidth; ++i) {
      cv::Point2d pointxy(lineIndex, i);
        //int _distance = static_cast<int>(depthImage.at<ushort>(pointxy));
        //line_distances.push_back(_distance);
    }

        bool stuck = false;
        for (int i = 0.4 * imageWidth; i < 0.6 * imageWidth; i++)
        {
          // printf("the range is: %f - %f \n",lms->ranges.size() * 0.4,lms->ranges.size() * 0.6);
          if (line_distances[i] < obstacleThreshold)
            {
            stuck = true;
            }
        }
        if (stuck)
        {
            //command.linear.x = 0.0;
            //command.angular.z = 0.5;
        }
        else
        {
            //command.linear.x = SPEED;
            //command.angular.z = 0.0;
        }   

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_line_extractor");
  ros::NodeHandle nh;
  ros::NodeHandle node;

  // Subscribe to the depth image topic
  ros::Subscriber depthImageSub = nh.subscribe("/aditof_roscpp/aditof_depth", 100, depthImageCallback);

  //ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/cmd_vel",10);
  
  /*
  command.linear.x = SPEED;
  command.linear.y = 0.0;
  command.linear.z = 0.0;
  command.angular.x = 0.0;
  command.angular.y = 0.0;
  command.angular.z = 0.0;

  */

  // Loop at 10Hz, publishing movement commands until we shut down.

  //pub.publish(command);
  ros::spin();
  return 0;
}