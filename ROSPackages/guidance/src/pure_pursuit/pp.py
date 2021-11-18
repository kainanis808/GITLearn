#!/usr/bin/env python

from math import sqrt, sin, cos, atan2, pi

class PurePursuit:

    def __init__(self, k = {'ku':1,
                            'kr':2}
                     , umax = 1):

        self.k = k    #proportionality constant
                           #[gain on u, gain on r]
        self.maxspeed = umax   #set maximum commanded speed limits
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

        self.nu_b[0] = self.k['ku']*e_u
        self.nu_b[1] = 0
        self.nu_b[2] = self.k['kr']*beta

    def limit_nu(self):
        for i in range(3):
            if self.nu_b[i] > self.maxspeed:
                self.nu_b[i] = self.maxspeed
            elif self.nu_b[i] < -self.maxspeed:
                self.nu_b[i] = -self.maxspeed
