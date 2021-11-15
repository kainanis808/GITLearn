#!/usr/bin/env python

import rospy
from turtlesim.srv import Kill, Spawn, TeleportAbsolute, SetPen
from std_srvs.srv import Empty
from math import pi

#rospy.init_node('config_window')

def config_window(req):
     #Clear everything
     reset_turtlesim = rospy.ServiceProxy('/reset',Empty)
     reset_turtlesim()
     kill_turtle = rospy.ServiceProxy('/kill',Kill)
     kill_turtle('turtle1')
     clear_screen = rospy.ServiceProxy('/clear',Empty)
     clear_screen()
     
     #Set circle parameters
     circle = rospy.get_param("/circle",{'center':{'x':5,'y':5},
                                         'radius': 4})
     center = circle['center']
     radius = circle['radius']
     #center = {'x':5,'y':5}
     #radius = 4
     
     #Spawn the follower on the circle center point right
     set_spawn = rospy.ServiceProxy('/spawn',Spawn)
     set_spawn(center['x'],center['y'],0,'follower')
     
     #Spawn a leader and use it to draw the circle center lines
     ## Determine the verticies of center lines to be drawn
     x = [center['x'],
          center['x'],
          center['x'],
          center['x']-radius,
          center['x']+radius]
     y = [center['y']-radius,
          center['y']+radius,
          center['y'],
          center['y'],
          center['y']]
     t = [0, 0, 0, 0, pi/2]
     
     ## Spawn the leader at the first vertex
     set_spawn(x[0],y[0],t[0],'leader')
     
     ## Move the leader to the other verticies to create the center lines
     set_pose_leader = rospy.ServiceProxy('/leader/teleport_absolute',
                                          TeleportAbsolute)
     for i in range(5):
         set_pose_leader(x[i],y[i],t[i])
     
     # Set the pen color of the leader to black and the follower to red
     set_pen_leader = rospy.ServiceProxy('/leader/set_pen',SetPen)
     set_pen_leader(0,0,0,2,0)
     set_pen_follower = rospy.ServiceProxy('/follower/set_pen',SetPen)
     set_pen_follower(255,0,0,2,0)

     return {"success": True, "message": "Window reconfigured"}
