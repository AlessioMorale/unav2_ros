#include <boost/asio/io_service.hpp>
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <memory>
#include <string>

#include "controller_manager/controller_manager.h"
#include "ros/ros.h"
#include "rosserial_server/serial_session.h"
#include "unav2_hardware/unav2_hardware.h"
#include <unav2_msgs/SystemStatus.h>
#include <sensor_msgs/BatteryState.h>


const std::string SYSTEM_TOPIC = "/unav2/status/system";
const std::string BATTERY_STATE_TOPIC = "/status/battery";

int param_battery_technology;
double param_capacity;
int param_cells;
std::unique_ptr<ros::Publisher> battery_state_pub;

typedef boost::chrono::steady_clock time_source;
void controlThread(ros::Rate rate, unav2_hardware::Unav2Hardware *unav2, controller_manager::ControllerManager *cm);

void controlThread(ros::Rate rate, unav2_hardware::Unav2Hardware *unav2, controller_manager::ControllerManager *cm) {
  time_source::time_point last_time = time_source::now();

  while (1) {
    time_source::time_point this_time = time_source::now();
    boost::chrono::duration<double> elapsed_duration = this_time - last_time;
    ros::Duration elapsed(elapsed_duration.count());
    last_time = this_time;

    unav2->copyJointsState();
    cm->update(ros::Time::now(), elapsed);
    unav2->publish();
    rate.sleep();
  }
}

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
  // Initialize ROS node.
  ros::init(argc, argv, "unav2hardware_node");
  unav2_hardware::Unav2Hardware unav2;
  ros::NodeHandle controller_nh("");

  controller_nh.param("battery_technology", param_battery_technology, (int)sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN);
  controller_nh.param("battery_cells", param_cells, (int)0);
  controller_nh.param("battery_capacity", param_capacity, (double)nan(""));

  controller_manager::ControllerManager cm(&unav2, controller_nh);
  boost::thread(boost::bind(controlThread, ros::Rate(50), &unav2, &cm));
  auto system_subscriber = controller_nh.subscribe(SYSTEM_TOPIC, 1, systemStateCallback);
  battery_state_pub = std::make_unique<ros::Publisher>(controller_nh.advertise<sensor_msgs::BatteryState>(BATTERY_STATE_TOPIC, 1, true));
  ros::spin();
  return 0;
}