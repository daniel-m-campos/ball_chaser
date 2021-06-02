#ifndef SRC_BALL_CHASER_SRC_PROCESS_IMAGE_H_
#define SRC_BALL_CHASER_SRC_PROCESS_IMAGE_H_

#include <sensor_msgs/Image.h>

#include "ros/ros.h"

enum class Location { kNone, kLeft, kMiddle, kRight };

Location LocateBall(const sensor_msgs::Image& img, int rgb_color = 255);

class ImageProcessor {
 public:
  explicit ImageProcessor(ros::NodeHandle& node_handle);

 private:
  ros::ServiceClient client_;
  ros::Subscriber sub_;
  void handle(const sensor_msgs::Image& img);
  void drive_robot(float lin_x, float ang_z);
};

#endif  // SRC_BALL_CHASER_SRC_PROCESS_IMAGE_H_
