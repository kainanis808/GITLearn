#!/usr/bin/env python

import rospy

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

from math import sqrt, sin, cos, atan2, pi

from config_window import config_window

class PurePursuit:

    def __init__(self):
        self.k = [1,2]     #proportionality constant
                           #[gain on u, gain on r]
        self.maxspeed = 2   #set maximum commanded speed limits
                            #u, v, r of the follower are all limited
                            #to plus or minus this value.
        self.pose_enu = {'L':{'x':0,'y':0,'psi':0},
                     'F':{'x':0,'y':0,'psi':0}}
        self.nu_b = [0,0,0]

    def set_pose(self, x, y, psi, vessel):
        self.pose_enu[vessel]['x'] = x
        self.pose_enu[vessel]['y'] = y
        self.pose_enu[vessel]['psi'] = psi

    def position_vector_enu(self):
        px = self.pose_enu['L']['x'] - self.pose_enu['F']['x']
        py = self.pose_enu['L']['y'] - self.pose_enu['F']['y']
        return [px, py]

    def enu_to_b(self,p_enu):
        psi = self.pose_enu['F']['psi']
        return [p_enu[0]*cos(psi)+p_enu[1]*sin(psi),
              -p_enu[0]*sin(psi)+p_enu[1]*cos(psi)]
    
    def mag(self, p):
        return sqrt(p[0]*p[0] + p[1]*p[1])

    def direction(self, p):
        return atan2(p[1],p[0])

    def set_nu(self):
        p_enu = self.position_vector_enu()
        p_b = self.enu_to_b(p_enu)

        e_u = p_b[0]
        beta = self.direction(p_b)

        self.nu_b[0] = self.k[0]*e_u
        self.nu_b[1] = 0
        self.nu_b[2] = self.k[1]*beta

    def limit_nu(self):
        for i in range(3):
            if self.nu_b[i] > self.maxspeed:
                self.nu_b[i] = self.maxspeed
            elif self.nu_b[i] < -self.maxspeed:
                self.nu_b[i] = -self.maxspeed


class PurePursuitROSWrapper:
    
    def __init__(self):
        # Initialize a Pure Pursuit path planning object
        self.pp = PurePursuit()

        # Create subscribers
        ## Topics
        rospy.Subscriber("/follower/pose", Pose, self.pose_follower)
        rospy.Subscriber("/leader/pose", Pose, self.pose_leader)
        ## Services
        rospy.Service("config_window", Trigger, config_window)

        #Create publishers
        self.publish_velocity = rospy.Publisher("/follower/cmd_vel",
                                           Twist, 
                                           queue_size=10)

    def pose_follower(self, msg):
        self.pp.set_pose(msg.x,msg.y,msg.theta,'F')
        self.pub_nu()

    def pose_leader(self, msg):
        self.pp.set_pose(msg.x,msg.y,msg.theta,'L')
        self.pub_nu()

    def pub_nu(self):
        self.pp.set_nu()
        self.pp.limit_nu()
        nu = Twist()
        nu.linear.x = self.pp.nu_b[0]
        nu.linear.y = self.pp.nu_b[1]
        nu.angular.z = self.pp.nu_b[2]

        #print(nu)

        self.publish_velocity.publish(nu)

if __name__=='__main__':
    rospy.init_node('pure_pursuit')

    pp_wrapper = PurePursuitROSWrapper()

    rospy.loginfo("Well it initilized, hopefully it is working.")
    rospy.spin()
