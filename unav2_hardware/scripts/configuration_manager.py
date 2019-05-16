#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from unav2_hardware.cfg import UnavBridgeConfig
from unav2_hardware.msg import BridgeConfig

pubBridgeConfig = rospy.Publisher('unav2/config/BridgeConfig', BridgeConfig, queue_size=10)

def callback(config, level):
    rospy.loginfo("""UnavBridgeConfig: Polarity {bridge_enable_polarity}, PWM Dead Zone {PWM_dead_zone}, PWM Frequency {PWM_frequency}, volt_offset {volt_offset}, volt_gain {volt_gain}, current_offset {current_offset}, current_gain {current_gain}""".format(**config))
    msg = BridgeConfig()
    msg.bridge_enable_polarity = config.bridge_enable_polarity
    msg.pwm_dead_zone = config.PWM_dead_zone
    msg.pwm_frequency = config.PWM_frequency
    msg.enable_polarity
    msg.current_offset
    msg.current_gain
    msg.volt_gain
    msg.volt_offset
    msg.transactionId
    pubBridgeConfig.publish(msg)
    return config

if __name__ == "__main__":
    rospy.init_node("unav2_configuration_manager", anonymous = False)



    srv = Server(UnavBridgeConfig, callback)
    rospy.spin()