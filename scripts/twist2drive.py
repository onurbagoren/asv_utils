#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import Twist
from kingfisher_msgs.msg import Drive

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.linear.x)
 
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.loginfo("Listening for Twist messages on topic=cmd_vel") 
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
