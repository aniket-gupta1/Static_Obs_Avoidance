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


import simplekml
from polycircles import polycircles





kml = simplekml.Kml()
file = open("/home/aannurag/Downloads/SDA/so_suas.txt", "r")    #change this for different laptops
i=0   #static obstacles
obstaclelist=list()
lines = file.readlines()

for i in lines:
    thisline=i.split()
    location=(thisline[0].latitude,thisline[1].longitude)
    r=(float(thisline[2].cylinder_radius)*0.3048)


    self.mpstate.map.add_object(mp_slipmap.SlipCircle('Obstacle Circle'+str(i), layer='Obstacle', latlon=location, radius=r, linewidth=-1, color=(255,0,0)))
    polygoncirclen = polycircles.Polycircle(latitude=s_o[i].latitude, longitude=s_o[i].longitude, radius=r, number_of_vertices=30)
    Name="Obstacle"+str(k)
    pol.append(kml.newpolygon(name=Name, outerboundaryis=polygoncirclen.to_kml()))
    pol[k].style.polystyle.color = \
                              simplekml.Color.changealphaint(200, simplekml.Color.red)
    k+=1
    file.write("%9.6f\t%9.6f\t%i\n" % (thisline[0].latitude,thisline[1].longitude,r));        
file.close()
kml.save("/home/aannurag/Downloads/SDA/my_obstacle.kml")      #change this for different laptops
print 'KML file created'