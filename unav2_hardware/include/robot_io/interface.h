#pragma once
#ifndef INTERFACE_H
#define INTERFACE_H
#include "abstract_connection.h"
#include "message_streaming.h"
#include <from_unav.pb.h>
#include <functional>
#include <google/protobuf/message.h>
#include <memory>
#include <stdint.h>
#include <to_unav.pb.h>
#include <vector>

namespace robot_io {

struct JointState {
  float position;
  float velocity;
  float effort;
};

enum class JointCommandMode { Failsafe = -1, Disabled = 0, Position = 1, Velocity = 2, Effort = 3 };
struct JointsCommand {
  std::vector<float> command;
  JointCommandMode mode;
};

class Interface {
private:
  std::shared_ptr<robot_io::connection::AbstractConnection> connection_;

  std::vector<std::function<void(const google::protobuf::Message &)>> message_callbacks_;
  std::vector<std::function<void(const std::vector<JointState> &)>> joint_state_callbacks_;

  void receive_callback(const std::vector<uint8_t> &incoming_packet) {
    std::vector<uint8_t> encoded(incoming_packet);
    std::vector<uint8_t> decoded_packet;
    while (encoded.size() > 0) {
      if (MessageStreaming::decode(encoded, decoded_packet)) {
        deserialize(decoded_packet);
      } else {
        break;
      }
    }
  }

  inline void deserialize(std::vector<uint8_t> &data) {
    MsgFromUnavUnion msg;
    if (msg.ParseFromArray(data.data(), data.size())) {
      if (msg.has_joints_state()) {
        handle_joints_state(msg.joints_state());
      } else {
        handle_message(msg);
      }
    }
  }

  inline void handle_message(google::protobuf::Message &message) {
    for (auto cb : message_callbacks_) {
      if (cb) {
        cb(message);
      }
    }
  }

  inline void handle_joints_state(MsgJointsState msg_joint_state) {
    std::vector<JointState> joints_state;
    auto msg_joints = msg_joint_state.joints();
    for (auto i : msg_joints) {
      JointState joint{.position = i.position(), .velocity = i.velocity(), .effort = i.effort()};
      joints_state.push_back(joint);
    }
    for (auto callback : joint_state_callbacks_) {
      if (callback) {
        callback(joints_state);
      }
    }
  }

public:
  explicit Interface(std::shared_ptr<robot_io::connection::AbstractConnection> connection) : connection_{connection} {
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    connection_->set_receive_callback(std::bind(&Interface::receive_callback, this, std::placeholders::_1));
  };

  ~Interface(){};

  void subscribe(std::function<void(const std::vector<JointState> &)> callback) {
    joint_state_callbacks_.push_back(callback);
  };

  void subscribe(std::function<void(const google::protobuf::Message &)> cb) {
    message_callbacks_.push_back(cb);
  };

  void publish_joint_command(JointsCommand commands) {
    static uint32_t seq = 0;
    seq++;
    MsgToUnavUnion msg;
    auto joints_command = msg.mutable_joint_command();
    joints_command->set_seq(seq);
    for (auto command : commands.command) {
      joints_command->add_command(command);
    }
    MsgJointCommand_JointCommandMode mode;

    switch (commands.mode) {
    case JointCommandMode::Failsafe:
      mode = MsgJointCommand_JointCommandMode::MsgJointCommand_JointCommandMode_failsafe;
      break;
    case JointCommandMode::Disabled:
      mode = MsgJointCommand_JointCommandMode::MsgJointCommand_JointCommandMode_disabled;
      break;
    case JointCommandMode::Position:
      mode = MsgJointCommand_JointCommandMode::MsgJointCommand_JointCommandMode_position;
      break;
    case JointCommandMode::Velocity:
      mode = MsgJointCommand_JointCommandMode::MsgJointCommand_JointCommandMode_velocity;
      break;
    case JointCommandMode::Effort:
      mode = MsgJointCommand_JointCommandMode::MsgJointCommand_JointCommandMode_effort;
      break;
    }

    joints_command->set_mode(mode);
    send_message(msg);
  }

  void send_message(google::protobuf::Message &msg) {
    std::string serialized_str;
    msg.SerializeToString(&serialized_str);
    std::vector<uint8_t> serialized(serialized_str.begin(), serialized_str.end());
    auto encoded = robot_io::MessageStreaming::encode(serialized);
    connection_->transmit(encoded);
  }
};
};     // namespace robot_io
#endif /* INTERFACE_H */
