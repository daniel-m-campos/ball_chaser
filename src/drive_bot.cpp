#include "ball_chaser/drive_bot.h"

#include <memory>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

DriveBotService::DriveBotService(std::unique_ptr<ros::Publisher> publisher)
    : publisher_{std::move(publisher)} {}

bool DriveBotService::handle(ball_chaser::DriveToTargetRequest &req,
                             ball_chaser::DriveToTargetResponse &res) {
  ROS_INFO("DriveToTargetRequest received - linear_x:%1.2f, angular_z:%1.2f",
           static_cast<float>(req.linear_x), static_cast<float>(req.angular_z));
  if (ros::ok()) {
    geometry_msgs::Twist motor_command;
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
    publisher_->publish(motor_command);
    res.msg_feedback = "DriveBotService Succeeded";
    ROS_INFO_STREAM(res.msg_feedback);
    return true;
  }
  res.msg_feedback = "ROS not OK";
  ROS_INFO_STREAM(res.msg_feedback);
  return false;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "drive_bot");
  ros::NodeHandle n;
  auto publisher = std::make_unique<ros::Publisher>(
      n.advertise<geometry_msgs::Twist>("/cmd_vel", 10));
  DriveBotService drive_bot_service{std::move(publisher)};
  auto service =
      n.advertiseService("/ball_chaser/command_robot", &DriveBotService::handle,
                         &drive_bot_service);

  ros::spin();

  return 0;
}
