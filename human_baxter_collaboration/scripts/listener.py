#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
from human_baxter_collaboration.msg import BaxterMoveitJoints, UnityTf
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

def listener():
    rospy.init_node('listener')
    rospy.Subscriber('/tf_static', TFMessage, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()