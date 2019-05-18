#!/usr/bin/env python

import rospy
from random import randint
from threading import Lock
from queue import Queue, Empty
from dynamic_reconfigure.server import Server
from std_msgs.msg import UInt32
from unav2_hardware.cfg import UnavBridgeConfig
from unav2_hardware.cfg import UnavDiagnosticConfig
from unav2_hardware.cfg import UnavEmergencyConfig
from unav2_hardware.cfg import UnavEncoderConfig
from unav2_hardware.msg import BridgeConfig
from unav2_hardware.msg import EncoderConfig


#   ["ConfigMessageName", MessageInstance(), [list_of_properties_to_copy], publisher_instance]
msgConfigs = [
    {"config": UnavEncoderConfig, "msg": EncoderConfig(), "propertynames": [["CPR", "cpr"], "position", ["z_index", "has_z_index"], "channels"],
     "publisher": rospy.Publisher('unav2/config/EncoderConfig', EncoderConfig, queue_size=10)},
    {"config": UnavBridgeConfig, "msg": BridgeConfig(), "propertynames": [["PWM_dead_zone", "pwm_dead_zone"], ["PWM_frequency", "pwm_frequency"], ["bridge_enable_polarity", "enable_polarity"], "current_offset", "current_gain", "volt_gain", "volt_offset"],
     "publisher": rospy.Publisher('unav2/config/BridgeConfig', BridgeConfig, queue_size=10)} 
]

srvConfigs = [
    {"configClass": UnavBridgeConfig, "namespace": "bridge"},
    {"configClass": UnavEncoderConfig, "namespace": "Encoder"}
]
messageQueue = []
ackQueue = Queue(maxsize=40) 
srv = []
MsgTTL = 30 # max retries for a message
messageQueueLock = Lock()

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
    for msgcfg in [msg for msg in msgConfigs if msg["config"] is cfgtype]:
        msg = {"msg": msgcfg["msg"], "ttl": MsgTTL}
        copyProperties(config, msg["msg"], msgcfg["propertynames"])
        msg["msg"].transactionId = randint(0, 2**32 - 1)
        #msgcfg["publisher"].publish(msg)
        
        messageQueueLock.acquire()
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
                    rospy.logwarn("Received Ack for a non existent transaction: " + str(ack))
                else:
                    for msg in msgs:
                        messageQueue.remove(msg)
                        rospy.logdebug("Transaction completed for message " + str(msg))
        except Empty:
            pass
        messageQueueLock.acquire()
        for msg in messageQueue :
            for cfg in [cfg for cfg in msgConfigs if cfg["msg"] == msg["msg"]]:
                rospy.logdebug("Sending message " + str(msg["msg"]) + ", ttl:" + str(msg["ttl"]))
                cfg["publisher"].publish(msg["msg"]) 
            msg["ttl"] = msg["ttl"] - 1
            if msg["ttl"] <= 0:
                rospy.logerr("No Acknowledge received for transaction " + str(msg["msg"].transactionId) + ", message discarded")
                messageQueue.remove(msg)
        messageQueueLock.release()

        rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node("unav2_configuration", anonymous=False)

        # Setup dynamic recofigurre callbacks
        [srv.append(Server(config["configClass"], bindCallback(config["configClass"]), config["namespace"]))
        for config in srvConfigs]
        
        rospy.Subscriber("ack", UInt32, configAckCallback)
        
        run_node()

    except rospy.ROSInterruptException:
        pass