#include <gtest/gtest.h>
#include <memory>
#include <robot_io/interface.h>
#include <robot_io/message_streaming.h>
#include <vector>

class MockConnection : public robot_io::connection::AbstractConnection {
public:
  MockConnection(){};
  bool connected = true;
  bool is_connected() override {
    return connected;
  }

  void set_receive_callback(std::function<void(const std::vector<uint8_t> &)> callback) override {
    callback_ = callback;
  };

  void transmit(std::vector<uint8_t> &data) override {
    std::vector copy(data);
    transmitted_data.push_back(copy);
  };
  void simulate_receive(const std::vector<uint8_t> &data) {
    callback_(data);
  };

  ~MockConnection() override{};

  std::vector<uint8_t> get_transmitted_data() {
    auto data = transmitted_data.front();
    transmitted_data.pop_back();
    return data;
  }

private:
  std::vector<std::vector<uint8_t>> transmitted_data;
  std::function<void(const std::vector<uint8_t> &)> callback_;
};

std::vector<uint8_t> encode_msg(google::protobuf::Message &msg) {
  std::string serialized_str;
  msg.SerializeToString(&serialized_str);
  std::vector<uint8_t> serialized(serialized_str.begin(), serialized_str.end());
  auto encoded = robot_io::MessageStreaming::encode(serialized);
  return encoded;
}

template <typename msg_type> msg_type decode_msg(std::vector<uint8_t> &input) {
  std::vector<uint8_t> serialized(input);
  std::vector<uint8_t> decoded;
  robot_io::MessageStreaming::decode(serialized, decoded);
  msg_type msg;
  msg.ParseFromArray(decoded.data(), decoded.size());
  return msg;
}

std::vector<uint8_t> prepare_joint_status_payload(std::vector<robot_io::JointState> joints) {
  MsgFromUnavUnion msg;
  auto joints_state = msg.mutable_joints_state();
  for (auto joint : joints) {
    auto j = joints_state->add_joints();
    j->set_effort(joint.effort);
    j->set_position(joint.position);
    j->set_velocity(joint.velocity);
  }
  return encode_msg(msg);
}

TEST(Robot_IO, interface_receive_joint_status) {

  std::shared_ptr<MockConnection> connection = std::make_shared<MockConnection>();
  robot_io::Interface interface(connection);

  std::vector<robot_io::JointState> last_joint_state;
  bool joint_state_received = false;
  auto joint_state_cb = [&](const std::vector<robot_io::JointState> &joints) {
    for (auto j : joints) {
      robot_io::JointState copy = j;
      last_joint_state.push_back(copy);
    }
    joint_state_received = true;
  };

  interface.subscribe(joint_state_cb);

  robot_io::JointState j1 = {.position = 2.0, .velocity = 3.0, .effort = 1.0};
  robot_io::JointState j2 = {.position = 8.0, .velocity = 7.0, .effort = 9.0};

  const std::vector<robot_io::JointState> test_values{j1, j2};

  auto encoded = prepare_joint_status_payload(test_values);

  connection->simulate_receive(encoded);

  ASSERT_TRUE(joint_state_received);

  ASSERT_EQ(last_joint_state.size(), test_values.size());

  for (size_t i = 0; i < last_joint_state.size(); i++) {
    ASSERT_EQ(last_joint_state[i].effort, test_values[i].effort);
    ASSERT_EQ(last_joint_state[i].position, test_values[i].position);
    ASSERT_EQ(last_joint_state[i].velocity, test_values[i].velocity);
  }
}

TEST(Robot_IO, interface_receive_other_messages) {
  std::shared_ptr<MockConnection> connection = std::make_shared<MockConnection>();
  robot_io::Interface interface(connection);

  std::vector<robot_io::JointState> last_joint_state;
  bool has_received_message = false;
  MsgFromUnavUnion received_message;

  auto message_rx_cb = [&](const google::protobuf::Message &message) {
    has_received_message = true;
    received_message.CopyFrom(message);
  };
  interface.subscribe(message_rx_cb);

  MsgFromUnavUnion msg;
  const std::string test_string = "this is a test string, i guess";
  const auto test_level = MsgLogger_LogLevel_critical;

  auto msg_logger = msg.mutable_logger();
  msg_logger->set_message(test_string.c_str());
  msg_logger->set_level(test_level);

  auto encoded = encode_msg(msg);
  connection->simulate_receive(encoded);

  ASSERT_TRUE(has_received_message);
  ASSERT_TRUE(received_message.has_logger());
}

TEST(Robot_IO, interface_send_joint_command) {
  std::shared_ptr<MockConnection> connection = std::make_shared<MockConnection>();
  robot_io::Interface interface(connection);

  robot_io::JointsCommand test_data = {
      .command{0.1, 0.2},
      .mode = robot_io::JointCommandMode::Velocity,
  };

  interface.publish_joint_command(test_data);

  auto received = connection->get_transmitted_data();

  auto message = decode_msg<MsgToUnavUnion>(received);

  ASSERT_TRUE(message.has_joint_command());
  auto joints_command = message.joint_command();
  ASSERT_EQ(joints_command.mode(), MsgJointCommand_JointCommandMode_velocity);
  ASSERT_EQ(joints_command.command_size(), 2);
  ASSERT_EQ(joints_command.command(0), test_data.command[0]);
  ASSERT_EQ(joints_command.command(1), test_data.command[1]);
}
