#!/usr/bin/env python

import rospy

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from math import sqrt, sin, cos, atan2, pi

class PurePursuit:

    def __init__(self):
        self.k = [1,2]     #proportionality constant
        self.pose = {'L':{'x':0,'y':0,'psi':0},
                     'F':{'x':0,'y':0,'psi':0}}
        self.nu = [0,0,0]

    def set_pose(self, x, y, psi, vessel):
        self.pose[vessel]['x'] = x
        self.pose[vessel]['y'] = y
        self.pose[vessel]['psi'] = psi

    def position_vector(self):
        px = self.pose['L']['x'] - self.pose['F']['x']
        py = self.pose['L']['y'] - self.pose['F']['y']

        pmag = self.mag([px,py])

        return [px/pmag, py/pmag]
        #return [px, py]
    
    def mag(self, p):
        return sqrt(p[0]*p[0] + p[1]*p[1])

    def direction(self, p):
        d = atan2(p[1],p[0])
        if d < 0:
            d = d+2*pi
        return d
        

    def set_nu(self):
        p = self.position_vector()
        #print(p)
        theta = self.direction(p)

        psi = self.pose['F']['psi']

        dxi = theta-psi

        #print([psi,theta,dxi])

        #print([psi, theta, dxi])

        self.nu[0] = self.k[0]*(p[0]*cos(psi) + p[1]*sin(psi))
        self.nu[1] = 0
        self.nu[2] = dxi*self.k[1]

        #print(self.nu)

        #limit all speeds to pm 1
        #for i in range(3):
        #    if self.nu[i] < -1:
        #        self.nu[i] = -1
        #    elif self.nu[i] > 1:
        #        self.nu[i] = 1

        #print(self.nu)
    
class PurePursuitROSWrapper:
    
    def __init__(self):
        #Initialize a Pure Pursuit path planning object
        self.pp = PurePursuit()

        #Create subscribers
        rospy.Subscriber("/follower/pose", Pose, self.pose_follower)
        rospy.Subscriber("/leader/pose", Pose, self.pose_leader)

        #Create publisher
        self.publish_velocity = rospy.Publisher("/follower/cmd_vel",
                                           Twist, 
                                           queue_size=10)

    def pose_follower(self, msg):
        self.pp.set_pose(msg.x,msg.y,msg.theta,'F')
        self.pp.set_nu()
        self.pub_nu()

    def pose_leader(self, msg):
        self.pp.set_pose(msg.x,msg.y,msg.theta,'L')
        self.pp.set_nu()
        self.pub_nu()

    def pub_nu(self):
        nu = Twist()
        nu.linear.x = self.pp.nu[0]
        nu.linear.y = self.pp.nu[1]
        nu.angular.z = self.pp.nu[2]

        #print(nu)

        self.publish_velocity.publish(nu)

if __name__=='__main__':
    rospy.init_node('pure_pursuit')

    pp_wrapper = PurePursuitROSWrapper()

    rospy.loginfo("Well it initilized, hopefully it is working.")
    rospy.spin()
