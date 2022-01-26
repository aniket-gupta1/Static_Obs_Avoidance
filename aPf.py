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
    img = np.zeros((col_s,row_s), dtype=np.uint8)
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



def fn(cn):
	fn1=f[cn[0]][cn[1]]
	return fn1


def neighbour_min(sn,dn):
    min=fn([sn[0]-1],[sn[1]-1])
    mn=list(sn)
    m=0
    n=0
    dn=list()
    for i in range(-1,2):
        for j in range (-1,2):
            if (i==0 and j==0):
                continue
            mn[0]=sn[0]+i
            mn[1]=sn[1]+j
            f=fn(mn)
            if(f<=min):
                min=f
                m=i
                n=j
    sn[0]=sn[0]+m
    sn[1]=sn[1]+n
    return(sn)

def mark_wo(wo):
	imgwo= np.ones((col_s,row_s), dtype=np.uint8)
	imgwo[wo[0]][wo[1]]=0
	edt=ndimage.morphology.distance_transform_edt(imgwo)
	#edt=edt.astype(int)
	return edt

scale_factor=int(raw_input("Enter scaling factor"))
flist=geofencedata("suas_geo.txt")
latdist,londist,latlim,lonlim,row_s,col_s=rows_and_cols(flist)
img,polyarray=array2image(row_s,col_s,flist)
cv2.imshow("img",img)
cv2.waitKey(0)
ol2=so_obstacle("so_suas.txt")
wpary2,wlist=waypoint("wp_suas.waypoints")
print (wpary2)
cv2.imshow("img",img)
cv2.waitKey(0)
Pathlist=list()








 #conversion to numPy array of 0 and 1
img2=np.array(img,dtype=np.int8)
img2=img/255

#print img2
#cv2.imshow("img2",img2)
#cv2.waitKey(0)
# gives nearest obstacle and its distance
node=wpary2[0]
dest_node=wpary2[1]
edts=ndimage.morphology.distance_transform_edt(img2)
edts=edts.astype("uint8")
cv2.imshow("i",edts)
edts=np.where(edts>20,0,edts)
#edts=(edts/np.amax(edts))*255

cv2.circle(edts,(node[0],node[1]),5, (100,100,100), -1)
cv2.imshow("img2",edts)
#cv2.waitKey(0) to int array
#print edts
print ("edts formed")



#towards destination wayoint
edtwo=np.zeros((col_s,row_s), dtype=np.uint8)
edtwo=mark_wo(dest_node)
edtwo=255-((edtwo/np.amax(edtwo))*255)

#print edtwo
att_re=(edtwo+edts)
att_re=att_re*img2
att_re=(att_re/np.amax(att_re))*255
att_re=att_re.astype("uint8")

print (att_re)
cv2.imshow("att",att_re)




cv2.circle(att_re,(dest_node[0],dest_node[1]),10, (100), -1)
cv2.imshow("att",att_re)
cv2.waitKey(0)



i=0

while (i<(len(wpary2)-1)):
	node=wpary2[i]
	dest_node=wpary2[i+1]
	print ("startnode",node)
	print ("destination node", dest_node)
	edtwo=mark_wo(dest_node)
	f=edtwo+(edts.transpose())
	f=ndimage.morphology.distance_transform_edt(f)
	while (node!=dest_node):
		node=neighbour_min(node,dest_node)
		print ("returned",node)
		cv2.circle(img,(node[0],node[1]), 0, (100,100,100), -1)
		cv2.imshow("img",img)
		cv2.waitKey(0)
		Pathlist.append(node)
	i=i+1
	
	








	

"""e=[[0 for i in range (0,row_s)] for j in range (0,col_s)]

for i in range(row_s):
    for j in range(col_s):
        e[i][j]=int(edts[i][j])
        


for i in range(len(wpary2)):
    cv2.circle(img,(wpary2[i][0],row_s-int(wpary2[i][1])), 2, (100,100,100), -1)
    
cv2.imshow("img",img)
cv2.waitKey(0)


for i in range(len(wpary2)-1):
    sn=list(wpary2[i])
    sn[1]=row_s-sn[1]
    dn=list(wpary2[i+1])
    dn[1]=row_s-dn[1]
    print sn, dn
    cn=sn
    while(cn!=dn):
        Pathlist.append(cn)
        cn=neighbour_min(cn,dn)
        cv2.circle(img,(cn[0],cn[1]), 0, (100,100,100), -1)
        cv2.imshow("img",img)
        cv2.waitKey(0)
    print "reached     --------------- --------------- ------------- ----------"       
    Pathlist.append(dn)
"""
print ("yeah yeah")

for i in range(len(Pathlist)):
	img[Pathlist[i][0]][Pathlist[i][1]]==100
cv2.imshow("im2",img)
cv2.waitKey(0)

