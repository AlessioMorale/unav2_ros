#!/usr/bin/env python

import rospy
from random import randint
from threading import Lock
from queue import Queue, Empty
from dynamic_reconfigure.server import Server
from std_msgs.msg import UInt32
from unav2_hardware.cfg import UnavBridgeConfig
from unav2_hardware.cfg import UnavEncoderConfig
from unav2_hardware.cfg import UnavLimitsConfig
from unav2_hardware.cfg import UnavMechanicalConfig
from unav2_hardware.cfg import UnavOperationConfig
from unav2_hardware.cfg import UnavPIDConfig
from unav2_hardware.cfg import UnavSafetyConfig


from unav2_hardware.msg import BridgeConfig
from unav2_hardware.msg import EncoderConfig
from unav2_hardware.msg import LimitsConfig
from unav2_hardware.msg import MechanicalConfig
from unav2_hardware.msg import OperationConfig
from unav2_hardware.msg import PIDConfig
from unav2_hardware.msg import SafetyConfig

# List all configuration sets and associated message to be send to unav
# namespace: dynamic_reconfigure namespace
# config: configuration type
# msg: message instance
# propertynames: list of property to copy from config to message or tuples when source/dest name differs
configurationSets = [
    {"namespace": "bridge", "config": UnavBridgeConfig, "msg": BridgeConfig(), 
        "propertynames": ["pwm_dead_time", "pwm_frequency", "bridge_enable_polarity", "bridge_disable_mode_outa", "bridge_disable_mode_outb", "current_offset", "current_gain", "volt_gain", "volt_offset"]},
    {"namespace": "encoder", "config": UnavEncoderConfig, "msg": EncoderConfig(), 
        "propertynames": ["cpr", "position", "has_z_index", "channels"]},
    {"namespace": "limits", "config": UnavLimitsConfig, "msg": LimitsConfig(),    
        "propertynames": ['position','velocity','current','effort','pwm']},
    {"namespace": "mechanical", "config": UnavMechanicalConfig, "msg": MechanicalConfig(),
        "propertynames": ['ratio','rotation0','rotation1','rotation2','rotation3']},
    {"namespace": "operation", "config": UnavOperationConfig, "msg": OperationConfig(),
        "propertynames": ['settings_command','operation_mode']},
    {"namespace": "pid", "config": UnavPIDConfig, "msg": PIDConfig(),
        "propertynames": ["vel_kp","vel_ki","vel_kd","vel_kaw","cur_kp","cur_ki","cur_kd","cur_kaw","vel_frequency","cur_frequency","cur_enable"]},
    {"namespace": "safety", "config": UnavSafetyConfig, "msg": SafetyConfig(),
        "propertynames": ["temp_warning","temp_critical","temp_timeout","temp_autorestore","cur_warning","cur_critical","cur_timeout","cur_autorestore","slope_time","bridge_off","timeout"]},
]


MsgTTL = 30 # max retries for a message
PublishersNamespace = "unav2/config"
messageQueue = []
ackQueue = Queue(maxsize=40) 
srv = []
messageQueueLock = Lock()
publishers = {}

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
    for msgcfg in [msg for msg in configurationSets if msg["config"] is cfgtype]:
        msg = {"msg": msgcfg["msg"], "ttl": MsgTTL}
        copyProperties(config, msg["msg"], msgcfg["propertynames"])
        msg["msg"].transactionId = randint(0, 2**32 - 1)
        #msgcfg["publisher"].publish(msg)
        
        messageQueueLock.acquire()
        [messageQueue.remove(m) for m in messageQueue if m["msg"] == msg["msg"]]
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
    ackQueue.put(data.data)    

def run_node():
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        try:
            while True: 
                ack = ackQueue.get(False, 0)
                msgs = [msg for msg in messageQueue if msg["msg"].transactionId == ack]
                if len(msgs) == 0:
                    rospy.logwarn("Received Ack for a non existing transaction: " + str(ack))
                else:
                    for msg in msgs:
                        messageQueue.remove(msg)
                        rospy.logdebug("Transaction completed for message " + str(msg))
        except Empty:
            pass
        messageQueueLock.acquire()
        for msg in messageQueue :
            for cfg in [cfg for cfg in configurationSets if cfg["msg"] == msg["msg"]]:
                rospy.logdebug("Sending message " + str(msg["msg"]) + ", ttl:" + str(msg["ttl"]))
                publishers[cfg["config"].__name__].publish(msg["msg"]) 
            msg["ttl"] = msg["ttl"] - 1
            if msg["ttl"] <= 0:
                rospy.logerr("No acknowledge received for transaction " + str(msg["msg"].transactionId) + ", message discarded")
                messageQueue.remove(msg)
        messageQueueLock.release()

        rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node("unav2_configuration", anonymous=False)

        
        for config in configurationSets:
            # Setup publishers
            publishers[config["config"].__name__] = rospy.Publisher(PublishersNamespace + "/" + config["namespace"], type(config["msg"]), queue_size=10)

            # Setup dynamic recofigurre callbacks
            srv.append(Server(config["config"], bindCallback(config["config"]), config["namespace"]))
        
        rospy.Subscriber("ack", UInt32, configAckCallback)
        
        run_node()

    except rospy.ROSInterruptException:
        pass