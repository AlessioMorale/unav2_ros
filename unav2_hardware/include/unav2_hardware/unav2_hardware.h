#pragma once

#include "boost/thread.hpp"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "realtime_tools/realtime_publisher.h"
#include "ros/ros.h"
#include "unav2_msgs/JointCommand.h"
#include "unav2_msgs/JointState.h"
#include "vector"
namespace unav2_hardware {

class Unav2Hardware : public hardware_interface::RobotHW {
public:
  Unav2Hardware();
  void copyJointsState();
  void publish();

private:
  void jointStateCallback(const unav2_msgs::JointStateConstPtr &msg);

  ros::NodeHandle nh_;
  ros::Subscriber jointstate_sub;
  realtime_tools::RealtimePublisher<unav2_msgs::JointCommand> jointcommand_pub;

  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::VelocityJointInterface velocity_joint_interface;

  typedef struct joint {
    // time stamp;
    double position;
    double velocity;
    double effort;
    double command;
  } joint_t;
  std::vector<joint_t> joints;
  // This pointer is set from the ROS thread.
  unav2_msgs::JointStateConstPtr jointstate_msg;
  boost::mutex jointstate_msg_mutex;
};

} // namespace unav2_hardware