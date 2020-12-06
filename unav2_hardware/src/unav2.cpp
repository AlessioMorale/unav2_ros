#include <memory>
#include <string>

#include "unav2_hardware/unav2_hardware.h"
#include "unav2_hardware/unav2_topics.h"
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <unav2_msgs/SystemStatus.h>

int param_battery_technology;
double param_capacity;
int param_cells;
std::unique_ptr<ros::Publisher> battery_state_pub;

void systemStateCallback(const unav2_msgs::SystemStatus &msg) {
  static int seq = 0;
  static auto previous_time = ros::Time::now();
  static double discharged = 0.0;
  auto time = ros::Time::now();

  sensor_msgs::BatteryState battery_state;
  battery_state.header.stamp = time;
  battery_state.header.seq = seq++;
  battery_state.voltage = msg.battery_voltage;
  battery_state.current = msg.battery_current;
  battery_state.capacity = param_capacity < 0 ? nan("") : param_capacity;
  battery_state.design_capacity = nan("");
  battery_state.charge = nan("");
  battery_state.percentage = nan("");

  double duration_hours = (time - previous_time).toSec() / 3600.0;
  previous_time = time;
  discharged -= battery_state.current * 1000 * duration_hours;
  if (param_capacity > 0) {
    battery_state.charge = param_capacity - discharged;
  }
  battery_state_pub->publish<sensor_msgs::BatteryState>(battery_state);
}

int main(int argc, char *argv[]) {
  const std::string name = "unav2_hardware";
  // Initialize ROS node.
  ros::init(argc, argv, name);
  ROS_WARN_NAMED(name, "Starting");

  ros::NodeHandle nh("");

  ros::AsyncSpinner spinner(3);
  spinner.start();
  ROS_INFO_NAMED(name, "Starting hardware interface");
  std::shared_ptr<unav2_hardware::Unav2Hardware> unav2(new unav2_hardware::Unav2Hardware(nh));

  nh.param("battery_technology", param_battery_technology, (int)sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN);
  nh.param("battery_cells", param_cells, (int)0);
  nh.param("battery_capacity", param_capacity, (double)0.0);

  auto system_subscriber = nh.subscribe(unav2_hardware::topics::system_topic, 1, systemStateCallback);
  battery_state_pub = std::make_unique<ros::Publisher>(nh.advertise<sensor_msgs::BatteryState>(unav2_hardware::topics::battery_state_topic, 1, true));

  ros::waitForShutdown();
  return 0;
}