#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from cage_msgs.msg import FeederCommand

class FeederSimulator:
    def __init__(self):
        self.pub = rospy.Publisher('/feeder/status', Bool, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher('/feeder/state', Bool, queue_size=1, latch=True)
        self.publish(False)
        rospy.Subscriber('/feeder/command', FeederCommand, self.callback, queue_size=5)
    def publish(self, enabled):
        self.pub.publish(enabled)
        self.state_pub.publish(enabled)
    def callback(self, msg):
        # Simulation actuator acknowledges immediately; hardware must implement its own feedback.
        rospy.loginfo('FEEDER %s: %s', 'ON' if msg.enable else 'OFF', msg.reason)
        self.publish(msg.enable)

if __name__ == '__main__':
    rospy.init_node('feeder_simulator_node'); FeederSimulator(); rospy.spin()
