#include "unav2_hardware/unav2_hardware.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/assign.hpp>
namespace unav2_hardware {

Unav2Hardware::Unav2Hardware() {
  std::string tmp;
  ros::V_string joint_names;
  nh_.getParam("JointNames", tmp);
  if (tmp.empty()) {
    tmp = "joint1,joint2";
  }
  boost::erase_all(tmp, " ");
  boost::split(joint_names, tmp, boost::is_any_of(","));

  joints.resize(joint_names.size());

  for (unsigned int i = 0; i < joint_names.size(); i++) {
    hardware_interface::JointStateHandle joint_state_handle(
        joint_names[i], &joints[i].position, &joints[i].velocity,
        &joints[i].effort);
    joint_state_interface.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(joint_state_handle,
                                                 &joints[i].command);
    velocity_joint_interface.registerHandle(joint_handle);
  }

  registerInterface(&joint_state_interface);
  registerInterface(&velocity_joint_interface);

  jointstate_sub =
      nh_.subscribe("feedback", 1, &Unav2Hardware::jointStateCallback, this);

  jointcommand_pub.init(nh_, "unav2/control/joint_cmd", 1);
  jointcommand_pub.msg_.command.push_back(0);
  jointcommand_pub.msg_.command.push_back(0);
}

void Unav2Hardware::copyJointsState() {
  boost::mutex::scoped_lock jointstate_msg_lock(jointstate_msg_mutex,
                                                boost::try_to_lock);
  if (jointstate_msg && jointstate_msg_lock &&
      jointstate_msg->position.size() > 0) {
    int statesize = jointstate_msg->position.size();

    for (int i = 0; (i < joints.size()) && (i < statesize); i++) {
      joints[i].position = static_cast<double>(jointstate_msg->position[i]);
      joints[i].velocity = static_cast<double>(jointstate_msg->velocity[i]);
      joints[i].effort = static_cast<double>(jointstate_msg->effort[i]);
    }
  }
}

void Unav2Hardware::publish() {
  if (jointcommand_pub.trylock()) {
    jointcommand_pub.msg_.mode = unav2_msgs::JointCommand::VELOCITY;
    jointcommand_pub.msg_.command[0] = joints[0].command;
    jointcommand_pub.msg_.command[1] = joints[1].command;
    jointcommand_pub.unlockAndPublish();
  }
}

void Unav2Hardware::jointStateCallback(
    const unav2_msgs::JointStateConstPtr &msg) {
  boost::mutex::scoped_lock lock(jointstate_msg_mutex);
  jointstate_msg = msg;
}

} // namespace unav2_hardware