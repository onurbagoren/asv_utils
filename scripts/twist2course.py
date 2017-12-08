#!/usr/bin/env python
# license removed for brevity

import rospy
import time
from geometry_msgs.msg import Twist
from kingfisher_msgs.msg import Drive
from kingfisher_msgs.msg import Course

class Node():
    def __init__(self,linear_scaling,angular_scaling):
        self.linear_scaling = linear_scaling
        self.angular_scaling = angular_scaling
        self.pub = None
        self.courseMsg =None 
        self.t0 = time.time()
        self.yaw = 0

    def callback(self,data):
        rospy.logdebug("RX: Twist "+rospy.get_caller_id())
        rospy.logdebug("\tlinear:")
        rospy.logdebug("\t\tx:%f,y:%f,z:%f"%(data.linear.x,
                                            data.linear.y,
                                            data.linear.z))
        rospy.logdebug("\tangular:")
        rospy.logdebug("\t\tx:%f,y:%f,z:%f"%(data.angular.x,
                                            data.angular.y,
                                            data.angular.z))
        # scaling factors
        linfac = self.linear_scaling
        angfac = self.angular_scaling
        self.courseMsg.speed = linfac*data.linear.x * 3.0
        #dt = time.time()-self.t0
        self.courseMsg.yaw += angfac*data.angular.z * 0.2  # fudge
        
        rospy.logdebug("TX: Course ")
        rospy.logdebug("\tspeed:%f, yaw:%f"%(self.courseMsg.speed,
                                             self.courseMsg.yaw))
        #rospy.loginfo("left: %f, right: %f"%(self.driveMsg.left,self.driveMsg.right))
        self.pub.publish(self.courseMsg)


if __name__ == '__main__':

    rospy.init_node('twist2course', anonymous=False)

    # ROS Parameters
    in_topic = rospy.get_param('~input_topic','cmd_vel')
    out_topic = rospy.get_param('~output_topic','cmd_course')
    # Scaling from Twist.linear.x to (left+right)
    linear_scaling = rospy.get_param('~linear_scaling',0.2)
    # Scaling from Twist.angular.z to (right-left)
    angular_scaling = rospy.get_param('~angular_scaling',0.05)

    rospy.loginfo("Subscribing to <%s>, Publishing to <%s>"%(in_topic,out_topic))
    rospy.loginfo("Linear scaling=%f, Angular scaling=%f"%(linear_scaling,angular_scaling))

    node=Node(linear_scaling,angular_scaling)

    # Publisher
    node.pub = rospy.Publisher(out_topic,Course,queue_size=10)
    node.courseMsg = Course()

    # Subscriber
    rospy.Subscriber(in_topic,Twist,node.callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
