#!/usr/bin/env python

import rospy

from turtlesim.msg import Pose
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Wrench

class Translate:
    
    def __init__(self):
        rospy.Subscriber("/follower/d_vel", Twist, self.pub_setpoint)
        rospy.Subscriber("/follower/cmd_vel", Twist, self.pub_state)
        rospy.Subscriber("/control_effort", Float64, self.pub_tau)

        #Create publishers
        self.setpoint_publisher = rospy.Publisher("/setpoint",
                                                  Float64, 
                                                  queue_size=10)
        self.state_publisher = rospy.Publisher("/state",
                                                  Float64, 
                                                  queue_size=10)
        self.tau_publisher = rospy.Publisher("/follower/tau",
                                             Wrench, 
                                             queue_size=10)

    def pub_setpoint(self,msg):
        self.setpoint_publisher.publish(msg.linear.x)

    def pub_state(self,msg):
        self.state_publisher.publish(msg.linear.x)

    def pub_tau(self,msg):
        tau = Wrench()
        tau.force.x = msg.data
        self.tau_publisher.publish(tau)
        
if __name__=='__main__':
    try:
        rospy.init_node('translator')
        translate = Translate()
        rospy.loginfo("Well it initilized, hopefully it is working.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
