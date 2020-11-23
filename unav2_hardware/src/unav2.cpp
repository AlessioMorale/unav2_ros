#include <boost/asio/io_service.hpp>
#include <boost/chrono.hpp>
#include <boost/thread.hpp>
#include <string>

#include "controller_manager/controller_manager.h"
#include "ros/ros.h"
#include "rosserial_server/serial_session.h"
#include "unav2_hardware/unav2_hardware.h"

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

int main(int argc, char *argv[]) {
  // Initialize ROS node.
  ros::init(argc, argv, "unav2hardware_node");
  unav2_hardware::Unav2Hardware unav2;

  ros::NodeHandle controller_nh("");
  controller_manager::ControllerManager cm(&unav2, controller_nh);
  boost::thread(boost::bind(controlThread, ros::Rate(50), &unav2, &cm));

  ros::spin();
  return 0;
}