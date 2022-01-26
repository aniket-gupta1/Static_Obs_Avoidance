#!/usr/bin/env python
'''
obstacle Module
'''

import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.mavproxy_map import mp_slipmap


class obstacle(mp_module.MPModule):
    def __init__(self, mpstate):
        """Initialise module"""
        super(obstacle, self).__init__(mpstate, "obstacle", "")
        self.status_callcount = 0
        self.boredom_interval = 2 # seconds
        self.last_bored = time.time()

        self.packets_mytarget = 0
        self.packets_othertarget = 0
        self.verbose = False

        self.obstacle_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
          ])
        self.add_command('obstacle', self.cmd_obstacle, "obstacle module", ['status','set (LOGSETTING)'])

    def usage(self):
        '''show help on command line options'''
        return "Usage: obstacle <status|set>"

    def cmd_obstacle(self, args):
        '''control behaviour of the module'''
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status())
        elif args[0] == "set":
            self.obstacle_settings.command(args[1:])
        else:
            print(self.usage())
	
	self.mpstate.map.add_object(mp_slipmap.SlipClearLayer('Obstacle'))

	#for plotting static obstacles, after reading co-ordinates from a file
	
    	obstacle_file = open("/home/jbhowmick/Desktop/obstacle.txt", "r")	#add correct location of file
	lines = obstacle_file.readlines()
	j = 0
	for i in lines:
 		thisline = i.split(" ")
                location = (float(thisline[0]),float(thisline[1]))
                r = ((float(thisline[2]))*0.3048)		#radius converted to meters
		self.mpstate.map.add_object(mp_slipmap.SlipCircle('Obstacle Circle'+str(j), layer='Obstacle', latlon=location, radius=r, linewidth=-1, color=(255,0,0)))
		j += 1
	obstacle_file.close()   	

    def status(self):
        '''returns information about module'''
        self.status_callcount += 1
        self.last_bored = time.time() # status entertains us
        return("status called %(status_callcount)d times.  My target positions=%(packets_mytarget)u  Other target positions=%(packets_mytarget)u" %
               {"status_callcount": self.status_callcount,
                "packets_mytarget": self.packets_mytarget,
                "packets_othertarget": self.packets_othertarget,
               })

    def boredom_message(self):
        if self.obstacle_settings.verbose:
            return ("Anurag bhadwa hai!!!")
        return ("Anurag bhadwa hai!!!")

    def idle_task(self):
        '''called rapidly by mavproxy'''
        now = time.time()
        if now-self.last_bored > self.boredom_interval:
            self.last_bored = now
            message = self.boredom_message()
            self.say("%s: %s" % (self.name,message))
            # See if whatever we're connected to would like to play:
            self.master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,
                                            message)

    def mavlink_packet(self, m):
        '''handle mavlink packets'''
        if m.get_type() == 'GLOBAL_POSITION_INT':
            if self.settings.target_system == 0 or self.settings.target_system == m.get_srcSystem():
                self.packets_mytarget += 1
            else:
                self.packets_othertarget += 1

def init(mpstate):
    '''initialise module'''
    return obstacle(mpstate)
