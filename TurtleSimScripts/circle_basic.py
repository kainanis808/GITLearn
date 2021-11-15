#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from math import pi, cos, sin

def kinematicCircleVelocity():
    vel = rospy.Publisher('/leader/cmd_vel',Twist,queue_size=10)
    rospy.init_node('circle_velocity')

    rate = rospy.Rate(1)

    circle = rospy.get_param("/circle",{'time':30, 'radius':4})
    #t_1rot = 30 #[s], time it takes to make one rotation
    omega = 2*pi/circle['time']

    R = circle['radius']

    u = R*omega

    t0 = rospy.get_time()

    nu = Twist()
    while not rospy.is_shutdown():
        t_now = rospy.get_time()
        t = t_now-t0
        alpha = omega*t
        phi = pi/2+alpha

        nu.linear.x = u
        nu.linear.y = 0
        nu.angular.z = omega

        vel.publish(nu)

        rate.sleep()

if __name__ == '__main__':
    try:
        kinematicCircleVelocity()
    except rospy.ROSInterruptException:
        pass
