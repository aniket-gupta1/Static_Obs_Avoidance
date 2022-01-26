#Prerequisite import
from geopy.point import Point
from geopy import distance
import numpy as np
from scipy import ndimage
import cv2
import math
import time
 

def geofencedata(Geofence_file):
	fence = open(Geofence_file,"r")
	flist = []
	for i in fence.readlines():
		thisline=i.split()
		fseq=[]
		fseq.append(float(thisline[0]))
		fseq.append(float(thisline[1]))
		flist.append(fseq)
	fence.close()
	return flist          

#GEO-FENCE  IMAGE   BOUNDARY
def rows_and_cols(flist):

	latlim=[]
	lonlim=[]
	alist=np.asarray(flist)
	latlim.append(float(min(alist[:,0])-0.001))
	latlim.append(float(max(alist[:,0])+0.001))
	lonlim.append(float(min(alist[:,1])-0.001))
	lonlim.append(float(max(alist[:,1])+0.001))

	p1 = Point(latlim[0],lonlim[0])
	p2 = Point(latlim[0],lonlim[1])
	p3 = Point(latlim[1],lonlim[1])
	p4 = Point(latlim[1],lonlim[0])
	d1=(distance.distance(p1,p2).meters)
	d2=(distance.distance(p3,p4).meters)
	londist=max(d1,d2)
	latdist=(distance.distance(p3,p2).meters)
	row_s=int(latdist/scale_factor)
	col_s=int(londist/scale_factor)
	if (row_s%scale_factor != 0):
		row_s+=1
	if (col_s%scale_factor != 0):
		col_s+=1
	return latdist,londist,latlim,lonlim,row_s,col_s



def gps2grid(lat,lon):
	row=row_s-(row_s*((lat-latlim[0])/(latlim[1]-latlim[0])))
	col=(col_s*((lon-lonlim[0])/(lonlim[1]-lonlim[0])))
	return [int(col),int(row)]
	
def grid2gps(row,col):
	lon=((col*(lonlim[1]-lonlim[0]))/col_s)+lonlim[0]
	lat=(((row_s-row)*(latlim[1]-latlim[0]))/row_s)+latlim[0]
	return (lat,lon)

def array2image(row_s,col_s,flist):
	img = np.zeros((row_s,col_s), dtype=np.uint8)
	polyarray=[]
	for j in range(0,len(flist)):
		polyarray.append(gps2grid(float(flist[j][0]),float(flist[j][1])))
		pts = np.array(polyarray, np.int32)
		cv2.polylines(img, [pts], True, (255),thickness=1)
		cv2.fillPoly(img,[pts],(255))
	return img,polyarray

def ft2me(x):
	return(x*0.3048)
	

def so_obstacle(so_file):
	so = open(so_file,"r")
	olist=list()
	
	lines = so.readlines()
	for i in lines:
		thisline = i.split()
		col,row=gps2grid(float(thisline[0]),float(thisline[1]))
		l3=list()
		l3.append(col)
		l3.append(row)
		radius=ft2me(float(thisline[2]))
		
		if radius<=scale_factor:
			img[col][row]=0
		else:
			radius=int(radius/scale_factor)
			if (radius%scale_factor != 0):
				radius+=1

		l3.append(radius)
		cv2.circle(img,(col,row), int(radius), (0,0,0), -1)
		olist.append(l3)
	so.close()
	return(olist)


def waypoint(wp_file):
	wp_i = open(wp_file,"r")
	y=list()
	j=0
	lines=wp_i.readlines()
	wparray=[]
	for i in lines:
		thisline = i.split()
		if j>=2:
			l=list()
			x=list()
			x.append(float(thisline[8]))
			x.append(float(thisline[9]))
			l=gps2grid(float(thisline[8]),float(thisline[9]))
			
			wparray.append(l)
			y.append(x)
		j+=1
	return wparray,y

def yc(Y):
	return(row_s-Y)

###################################################################edited below here#############################################

def flor(x):                       # fuction returns floor value of a float in integer eg -12.10000034  to -13 
	return(int(math.floor(x)))

def sign(number):                  # function returns the sign of the number  
    if(number<0):
        return -1
    elif(number>0):
        return 1 
    else:
    	return 0
          

def mark_wo(wo):                   # fuction marks the given Point "wo" on the nP array and returns its morPhological function

	imgwo=np.ones((row_s,col_s), dtype=np.uint8)
	imgwo[wo[1]][wo[0]]=0
	edt=ndimage.morphology.distance_transform_edt(imgwo)
	return edt

#--------------------------------------------------------------------------------------
scale_factor=int(raw_input("Enter scaling factor"))              # this Portion creates a black and white image "img" with geofence and obstacles  marked on it      
flist=geofencedata("suas_geo.txt")                           
latdist,londist,latlim,lonlim,row_s,col_s=rows_and_cols(flist)
img,polyarray=array2image(row_s,col_s,flist)
cv2.imshow("img",img)
cv2.waitKey(0)
ol2=so_obstacle("so_suas.txt")
wpary2,wlist=waypoint("wp_suas.waypoints")
cv2.imshow("img",img)
cv2.waitKey(0)                                                   # wPary 2 is the list of wayPoints in [column][row] format
#-------------------------------------------------------------------------------------


img2=np.array(img,dtype=np.int8)                         
img2=img/255                                               # converts the image to 0 and 1 with "zero" rePresenting obstacles and geofence boundary and one rePresenting safe Path
edts=ndimage.morphology.distance_transform_edt(img2)       # ceeates the morPhological image of the rePulsive 


np.set_printoptions(threshold=np.inf,linewidth=np.inf)     # ruses the whole terminal to Print the array


i=0
while (i<(len(wpary2)-1)):                                 # wPary 2 is the list of wayPoints in [column][row] format
	node=wpary2[1]                                         # ith node as start node
	dest_node=wpary2[1+1]                                  # i+1 th node as destination node

	print ("startnode",node)
	print ("destination node", dest_node)

	edtwo=np.zeros((row_s,col_s))                          
	edtwo=mark_wo(dest_node)                                   # edtwo is the morPhological function for the attractive function
	cv2.circle(img,(node[0],node[1]), 5, (100,100,100), -1)                  # only for visibility of start node
	cv2.circle(img,(dest_node[0],dest_node[1]), 5, (100,100,100), -1)        # only for visibility of end node
	cv2.imshow("img",img)
	cv2.waitKey(0)

	dr=(edts/100.0) + 1
	d0 = 2.0
	nu =100.0
	repulsive=nu*((1/dr-1/d0)**2) 
	repulsive=np.where(dr>2.0,0,repulsive)             # array storing rePulsive values


	xi = 1/1000.0
	attractive=(edtwo**2)*xi                               # array storing attractive values
	

	f = attractive+repulsive                                # net array


	

	fx=(np.array(np.gradient(-f,axis=1)))                   # gradient along x axis 
	fy=(np.array(np.gradient(-f,axis=0)))                   # graddient along y axis

	while ([flor(node[1]),flor(node[0])]!=dest_node):       # works till current node is not equal to dest node
		z=flor(node[0])
		w=flor(node[1])

		delta = [fx[w][z],fy[w][z]]                         # delta stores gradient values of x and y axis
		print "reulsive =",repulsive[w][z]
		print "attractive =",attractive[w][z]
		print "f =",f[w][z]

		print("delta is ",delta)
		mag =delta/np.linalg.norm(delta)                    # normalisation of delta values
		print mag[0],mag[1]
		
		node[0]=node[0]+mag[0]                              # normalized values gets added to the current node  ( till now all values are in float)
		node[1]=node[1]+mag[1] 

		print "returned",flor(node[1]),flor(node[0])
		cv2.circle(img,(flor(node[0]),flor(node[1])), 0, (100,100,100), -1)    # marks the returned node in the image 
		cv2.imshow("img",img)
		cv2.waitKey(0)
	i=i+1

print ("yeah yeah")

#### bhai abhi toh saara thik chal rha hai values gradients se hi uPdate ho rhi hai bs ab tumhe tune karna hai rePulsive aur attractive




