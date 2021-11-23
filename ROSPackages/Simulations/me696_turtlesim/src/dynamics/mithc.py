#!/usr/bin/env python

class MITHoverCraftDynamics:

    def __init__(self): 
       self.dt = 1.0/10.0
       self.nu0 =  {'u':0, 'v':0, 'r':0}
       self.nu1 = self.nu0
       self.tau =  {'X':0, 'Y':0, 'N':0}

    def set_dt(self,dt):
        self.dt = dt

    def set_tau(self,tau):
        self.tau = tau

    def update_nu(self):
        self.nu1['u'] = self.nu0['u']*(1-self.dt) + self.dt*self.tau['X']
        self.nu1['v'] = self.nu0['v'] + self.dt*self.tau['Y']
        self.nu1['r'] = self.nu0['r'] + self.dt*self.tau['N']

        self.nu0 = self.nu1

        return self.nu1
