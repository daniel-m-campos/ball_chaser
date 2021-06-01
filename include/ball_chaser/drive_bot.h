#ifndef SRC_DRIVEBOTNODE_H
#define SRC_DRIVEBOTNODE_H

#include "ball_chaser/DriveToTarget.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

class DriveBotService {
 public:
  explicit DriveBotService(std::unique_ptr<ros::Publisher> publisher);
  bool handle(ball_chaser::DriveToTargetRequest &req,
              ball_chaser::DriveToTargetResponse &res);

 private:
  std::unique_ptr<ros::Publisher> publisher_;
};

#endif  // SRC_DRIVEBOTNODE_H
