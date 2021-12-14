#!/usr/bin/env python

import rospy
import sys
from std_srvs.srv import Empty
from std_msgs.msg import String
from std_msgs.msg import Float32

#test if we can use this service script
check_topic = rospy.get_published_topics()
#print(check_topic)

#test if we can get the parameters
check_param = rospy.get_param_names()
#print(check_param)

#test if we can change multiple things
new_fwd_kp = sys.argv[1]
new_fwd_ki = sys.argv[2]
new_fwd_kd = sys.argv[3]

new_yawRate_kp = sys.argv[4]
new_yawRate_ki = sys.argv[5]
new_yawRate_kd = sys.argv[6]

set_fwd_kp = rospy.set_param('/controller/fwd_vel/kp',new_fwd_kp)
set_fwd_ki = rospy.set_param('/controller/fwd_vel/ki',new_fwd_ki)
set_fwd_kd = rospy.set_param('/controller/fwd_vel/kd',new_fwd_kd)

set_yawRate_kp = rospy.set_param('/controller/yaw_rate/kp',new_yawRate_kp)
set_yawRate_ki = rospy.set_param('/controller/yaw_rate/ki',new_yawRate_ki)
set_yawRate_kd = rospy.set_param('/controller/yaw_rate/kd',new_yawRate_kd)

#for debug process
check_fwd_kp = rospy.get_param('/controller/fwd_vel/kp')
check_fwd_ki = rospy.get_param('/controller/fwd_vel/ki')
check_fwd_kd = rospy.get_param('/controller/fwd_vel/kd')
check_yawRate_kp = rospy.get_param('/controller/yaw_rate/kp')
check_yawRate_ki = rospy.get_param('/controller/yaw_rate/ki')
check_yawRate_kd = rospy.get_param('/controller/yaw_rate/kd')

#resetting gazebo
rospy.wait_for_service('/gazebo/reset_world')
reset = rospy.ServiceProxy('/gazebo/reset_world', Empty)
reset()

#santiy check
print("Forward Velocity PID gains updated to be " + "P: " + new_fwd_kp + " " + "I: " + new_fwd_ki + " " + "D: " + new_fwd_kd + " ")
print("Yaw Rate PID gains updated to be " + "P: " + new_yawRate_kp + " " + "I: " + new_yawRate_ki + " " + "D: " + new_yawRate_kd + " ")
print("Gazebo world has been restarted")
