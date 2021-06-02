#include "ball_chaser/process_image.h"

#include <sensor_msgs/Image.h>

#include "ball_chaser/DriveToTarget.h"
#include "ros/ros.h"

Location LocateBall(const sensor_msgs::Image& img, int rgb_color) {
  auto Equal = [](int a, int b, int c, int d) {
    return (a == b) && (b == c) && (c == d);
  };
  auto i = 0;
  auto channels = 3;
  for (; i < img.data.size(); i += channels) {
    if (Equal(rgb_color, img.data[i], img.data[i + 1], img.data[i + 2])) break;
  }
  if (i == img.data.size()) return Location::kNone;
  auto column = (i / channels) % img.width;
  auto norm_horizontal_distance =
      static_cast<float>(column) / static_cast<float>(img.width);
  if (norm_horizontal_distance < 0.333) {
    return Location::kLeft;
  } else if (norm_horizontal_distance < 0.666) {
    return Location::kMiddle;
  } else {
    return Location::kRight;
  }
}

ImageProcessor::ImageProcessor(ros::NodeHandle& node_handle)
    : client_{node_handle.serviceClient<ball_chaser::DriveToTarget>(
          "/ball_chaser/command_robot")},
      sub_{node_handle.subscribe("/camera/rgb/image_raw", 10,
                                 &ImageProcessor::handle, this)}

{}

void ImageProcessor::drive_robot(float lin_x, float ang_z) {
  ROS_INFO_STREAM("Requesting DriveToTarget");
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;
  if (not client_.call(srv)) ROS_ERROR("Request to DriveToTarget failed.");
}

void ImageProcessor::handle(const sensor_msgs::Image& img) {
  float lin_x = 0.0;
  float ang_z = 0.0;
  switch (LocateBall(img)) {
    case Location::kLeft: {
      lin_x = 0.0;
      ang_z = 0.5;
      break;
    }
    case Location::kRight: {
      lin_x = 0.0;
      ang_z = -0.5;
      break;
    }
    case Location::kMiddle: {
      lin_x = 1.0;
      ang_z = 0.0;
    }
    case Location::kNone:
      break;
  }
  drive_robot(lin_x, ang_z);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;
  ImageProcessor image_processor(n);
  ros::spin();
  return 0;
}
