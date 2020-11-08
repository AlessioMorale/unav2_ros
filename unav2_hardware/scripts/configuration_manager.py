#!/usr/bin/env python3

import rospy
from random import randint
from threading import Lock
from queue import Queue, Empty
from dynamic_reconfigure.server import Server
from std_msgs.msg import UInt32
from unav2_hardware.cfg import UnavBridgeConfig
from unav2_hardware.cfg import UnavEncoderConfig
from unav2_hardware.cfg import UnavMechanicalConfig
from unav2_hardware.cfg import UnavOperationConfig
from unav2_hardware.cfg import UnavPIDConfig
from unav2_hardware.cfg import UnavSafetyConfig


from unav2_msgs.msg import BridgeConfig
from unav2_msgs.msg import EncoderConfig
from unav2_msgs.msg import MechanicalConfig
from unav2_msgs.msg import OperationConfig
from unav2_msgs.msg import PIDConfig
from unav2_msgs.msg import SafetyConfig

# List all configuration sets and associated message to be send to unav
# namespace: dynamic_reconfigure namespace
# config: configuration type
# msg: message instance
# propertynames: list of property to copy from config to message or tuples when source/dest name differs
configurationSets = [
    {"namespace": "bridge", "config": UnavBridgeConfig, "msg": BridgeConfig(),
        "propertynames": ["pwm_dead_time", "pwm_frequency"]},
    {"namespace": "encoder", "config": UnavEncoderConfig, "msg": EncoderConfig(),
        "propertynames": ["cpr", "position", "has_z_index", "channels", "invert0", "invert1"]},
    {"namespace": "mechanical", "config": UnavMechanicalConfig, "msg": MechanicalConfig(),
        "propertynames": ['ratio', 'rotation0', 'rotation1']},
    {"namespace": "operation", "config": UnavOperationConfig, "msg": OperationConfig(),
        "propertynames": ['operation_mode', 'reset_to_dfu']},
    {"namespace": "pid", "config": UnavPIDConfig, "msg": PIDConfig(),
        "propertynames": ["velocity_kp", "velocity_ki", "velocity_kd", "velocity_kaw", "velocity_frequency", "current_kp", "current_ki", "current_kd", "current_kaw", "current_frequency", "current_enable", 'pid_debug', ]},
    {"namespace": "safety", "config": UnavSafetyConfig, "msg": SafetyConfig(),
        "propertynames": ["temp_warning", "temp_limit", "temp_timeout", "temp_autorestore", 
        "current_warning", "current_limit", "current_timeout", "current_autorestore", 
        "position_limit", "velocity_limit", "max_acceleration", "max_deceleration",
        "pwm_limit", "error_limit",
        "slope_time", "bridge_off", "timeout"]},
]

ACK_CODE_RESENDALL = 0xFFFFFFFF
MsgTTL = -1  # max retries for a message
rootNamespace = "unav2"
PublishersNamespace = rootNamespace + "/config"
ackNamespace = rootNamespace + "/status/ack"
messageQueue = []
ackQueue = Queue(maxsize=40)
srv = []
messageQueueLock = Lock()
publishers = {}
configurations = {}

def copyProperties(source, dest, propertynames):
    for p in propertynames:
        if isinstance(p, list):
            pin = p[0]
            pout = p[1]
        else:
            pin = p
            pout = p
        v = getattr(source, pin)
        setattr(dest, pout, v)


def publishConfig(cfgtype, config):
    configurations[cfgtype.__name__] = config
    for msgcfg in [msg for msg in configurationSets if msg["config"] is cfgtype]:
        msg = {"msg": msgcfg["msg"], "ttl": MsgTTL}
        copyProperties(config, msg["msg"], msgcfg["propertynames"])
        msg["msg"].transactionId = randint(0, 2**32 - 1)
        # msgcfg["publisher"].publish(msg)

        messageQueueLock.acquire()
        [messageQueue.remove(m)
         for m in messageQueue if m["msg"] == msg["msg"]]
        messageQueue.append(msg)
        messageQueueLock.release()

        rospy.logdebug("enqueued message " + str(msg["msg"].transactionId))
        return
    rospy.logwarn(
        "Configuration not found for Dynamic Config message " + str(cfgtype))


def bindCallback(msgtype):
    def func1(*args):
        config = args[0]
        #level = args[1]
        publishConfig(msgtype, config)
        return config
    func1.__name__ = "callbackConfig" + msgtype.__name__
    return func1


def configAckCallback(data):
    rospy.logdebug("Received ack" + str(data))
    if data.data == ACK_CODE_RESENDALL:
        rospy.loginfo("Received request to resend all configurations")
        for key in configurations.keys():
            rospy.logdebug("Resending ", key)
            m = [msg for msg in configurationSets if msg["config"].__name__ == key]
            conf_type = m[0]["config"]
            conf = configurations[key]
            publishConfig(conf_type, conf)
    else:
        ackQueue.put(data.data)


def run_node():
    rate = rospy.Rate(5)  # 5Hz
    while not rospy.is_shutdown():
        try:
            while True:
                ack = ackQueue.get(False, 0)
                msgs = [
                    msg for msg in messageQueue if msg["msg"].transactionId == ack]
                if len(msgs) == 0:
                    rospy.logwarn(
                        "Received Ack for a non existing transaction: " + str(ack))
                else:
                    for msg in msgs:
                        messageQueue.remove(msg)
                        rospy.logdebug(
                            "Transaction completed for message " + str(msg))
        except Empty:
            pass
        messageQueueLock.acquire()
        for msg in messageQueue:
            for cfg in [cfg for cfg in configurationSets if cfg["msg"] == msg["msg"]]:
                rospy.logdebug("Sending message " +
                               str(msg["msg"]) + ", ttl:" + str(msg["ttl"]))
                publishers[cfg["config"].__name__].publish(msg["msg"])
            if MsgTTL > 0:
                msg["ttl"] = msg["ttl"] - 1
                if msg["ttl"] <= 0:
                    rospy.logerr("No acknowledge received for transaction " +
                                str(msg["msg"].transactionId) + ", message discarded")
                    messageQueue.remove(msg)
        messageQueueLock.release()

        rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("unav2_configuration", anonymous=False)
        MsgTTL = int(rospy.get_param('~msgttl', '10'))

        for config in configurationSets:
            # Setup publishers
            publishers[config["config"].__name__] = rospy.Publisher(
                PublishersNamespace + "/" + config["namespace"], type(config["msg"]), queue_size=10)

            # Setup dynamic recofigurre callbacks
            srv.append(Server(config["config"], bindCallback(
                config["config"]), config["namespace"]))

        rospy.Subscriber(ackNamespace, UInt32, configAckCallback)

        run_node()

    except rospy.ROSInterruptException:
        pass
