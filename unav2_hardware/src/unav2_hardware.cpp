#include "unav2_hardware/unav2_hardware.h"
#include "unav2_hardware/unav2_topics.h"
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#define _USE_MATH_DEFINES
#include <cmath>

namespace unav2_hardware {

Unav2Hardware::Unav2Hardware(ros::NodeHandle &nh) : nh_{nh} {
  std::string tmp{"joint1,joint2"};
  std::vector<std::string> joint_names;

  nh_.getParam("joints", joint_names);
  nh_.param("controller_rate", rate_, (double)50.0);

  joints_.resize(joint_names.size());
  jointcommand_pub_.init(nh, topics::joint_command, 1);
  jointcommand_pub_.msg_.command.resize(joint_names.size());

  for (unsigned int i = 0; i < joint_names.size(); i++) {
    auto joint_name = joint_names[i];
    hardware_interface::JointStateHandle joint_state_handle(joint_name, &joints_[i].position, &joints_[i].velocity, &joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(joint_state_handle, &joints_[i].velocity_command);

    joint_limits_interface::JointLimits limits;
    joint_limits_interface::SoftJointLimits softLimits;

    if (getJointLimits(joint_name, nh_, limits) == false) {
      ROS_ERROR_STREAM("Cannot set joint limits for " << joint_name);
    } else {
      joint_limits_interface::VelocityJointSoftLimitsHandle jointLimitsHandle(joint_handle, limits, softLimits);
      velocity_joint_limits_interface_.registerHandle(jointLimitsHandle);
    }

    velocity_joint_interface_.registerHandle(joint_handle);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);
  registerInterface(&velocity_joint_limits_interface_);


  controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
  ros::Duration update_freq = ros::Duration(1.0 / rate_);
  non_realtime_loop_ = nh_.createTimer(update_freq, &Unav2Hardware::update, this);
  jointstate_sub_ = nh.subscribe(topics::joint_status, 1, &Unav2Hardware::joint_state_callback, this);
}

void Unav2Hardware::read() {
  if (const auto lock = std::unique_lock{jointstate_msg_mutex_, std::try_to_lock}) {
    if (jointstate_msg_ && lock.owns_lock() && jointstate_msg_->position.size() > 0) {
      int statussize = jointstate_msg_->position.size();
      for (int i = 0; i < joints_.size(); i++) {
        joints_[i].position = static_cast<double>(jointstate_msg_->position[i % statussize] * (2 * M_PI));
        joints_[i].velocity = static_cast<double>(jointstate_msg_->velocity[i % statussize] * (2 * M_PI));
        joints_[i].effort = static_cast<double>(jointstate_msg_->effort[i % statussize]);
      }
    }
  }
}

void Unav2Hardware::update(const ros::TimerEvent &e) {
  elapsed_time_ = ros::Duration(e.current_real - e.last_real);
  read();
  controller_manager_->update(ros::Time::now(), elapsed_time_);
  write();
}

void Unav2Hardware::write() {
  if (jointcommand_pub_.trylock()) {
    jointcommand_pub_.msg_.mode = unav2_msgs::JointCommand::VELOCITY;
    for (int i = 0; i < joints_.size(); i++) {
      jointcommand_pub_.msg_.command[i] = (float)joints_[i].velocity_command / (2 * M_PI);
    }
    jointcommand_pub_.unlockAndPublish();
  }
}

void Unav2Hardware::joint_state_callback(const unav2_msgs::JointStateConstPtr &msg) {
  const std::lock_guard<std::mutex> lock(jointstate_msg_mutex_);
  jointstate_msg_ = msg;
}

} // namespace unav2_hardware