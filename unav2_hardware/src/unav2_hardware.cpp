#include "unav2_hardware/unav2_hardware.h"
#include "unav2_hardware/unav2_topics.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/assign.hpp>
#define _USE_MATH_DEFINES
#include <cmath>

namespace unav2_hardware {

Unav2Hardware::Unav2Hardware() {
  std::string tmp{"joint1,joint2"};
  std::vector<std::string> joint_names;
  nh.getParam("joint_names", tmp);

  boost::erase_all(tmp, " ");
  boost::split(joint_names, tmp, boost::is_any_of(","));

  joints.resize(joint_names.size());
  jointcommand_pub.init(nh, topics::joint_command, 1);
  jointcommand_pub.msg_.command.resize(joint_names.size());

  for (unsigned int i = 0; i < joint_names.size(); i++) {
    hardware_interface::JointStateHandle joint_state_handle(joint_names[i], &joints[i].position, &joints[i].velocity, &joints[i].effort);
    joint_state_interface.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(joint_state_handle, &joints[i].command);
    velocity_joint_interface.registerHandle(joint_handle);
  }

  registerInterface(&joint_state_interface);
  registerInterface(&velocity_joint_interface);

  jointstate_sub = nh.subscribe(topics::joint_status, 1, &Unav2Hardware::joint_state_callback, this);
}

void Unav2Hardware::read() {
  if (const auto lock = std::unique_lock{jointstate_msg_mutex, std::try_to_lock}) {
    if (jointstate_msg && lock.owns_lock() && jointstate_msg->position.size() > 0) {
      int statussize = jointstate_msg->position.size();
      for (int i = 0; i < joints.size(); i++) {
        joints[i].position = static_cast<double>(jointstate_msg->position[i % statussize] * (2 * M_PI));
        joints[i].velocity = static_cast<double>(jointstate_msg->velocity[i % statussize] * (2 * M_PI));
        joints[i].effort = static_cast<double>(jointstate_msg->effort[i % statussize]);
      }
    }
  }
}

void Unav2Hardware::write() {
  if (jointcommand_pub.trylock()) {
    jointcommand_pub.msg_.mode = unav2_msgs::JointCommand::VELOCITY;
    for (int i = 0; i < joints.size(); i++) {
      jointcommand_pub.msg_.command[i] = (float)joints[i].command / (2 * M_PI);
    }
    jointcommand_pub.unlockAndPublish();
  }
}

void Unav2Hardware::joint_state_callback(const unav2_msgs::JointStateConstPtr &msg) {
  const std::lock_guard<std::mutex> lock(jointstate_msg_mutex);
  jointstate_msg = msg;
}

} // namespace unav2_hardware