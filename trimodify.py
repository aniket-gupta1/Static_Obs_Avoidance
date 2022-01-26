 #Prerequisite import
from geopy.point import Point
from geopy import distance
import numpy as np
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

def array2image(row_s,col_S,flist):
    img = np.zeros((col_s,row_s), dtype=np.uint8)
    polyarray=[]
    for j in range(len(flist)):
        polyarray.append(gps2grid(float(flist[j][0]),float(flist[j][1])))
    pts = np.array(polyarray, np.int32)
    cv2.polylines(img, [pts], True, (255),thickness=1)
    cv2.fillPoly(img,[pts],(255))
    print polyarray
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
            radius=0
        else:
            radius=int(radius/scale_factor)
            if (radius%scale_factor != 0):
                radius+=1
        l3.append(radius)
        cv2.circle(img,(col,row), radius, (0,0,0), -1)
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
        if j>=1:
            l=list()
            x=list()
            x.append(float(thisline[8]))
            x.append(float(thisline[9]))
            l=gps2grid(float(thisline[8]),float(thisline[9]))
            cv2.circle(img,(l[0],l[1]), 2, (0,0,0), -1)
            wparray.append(l)
            y.append(x)
        j+=1
    return wparray,y

def yc(Y):
    return(row_s-Y)

    
def no_of_obstacles(snode,dnode,a):
    ol3=list()
    ol4=list()
    delx=dnode[0]-snode[0]
    dely=dnode[1]-snode[1]
    print "[[[[[",snode,dnode,"]]]]"
    d4=math.sqrt((dnode[1]-snode[1])**2+(dnode[0]-snode[0])**2)
    print "d4======",d4
    for i in range(len(ol2)):
        del2x=ol2[i][0]-snode[0]
        del2y=ol2[i][1]-snode[1]
        if((del2x>0)and(delx<0)):
            continue
        elif((del2y>0)and(dely<0)):
            continue
        elif((del2x<0)and(delx>0)):
            continue
        elif((del2y<0)and(dely>0)):
            continue
        else:
            ol3.append(ol2[i])
    for i in range (len(ol3)):
        dist=abs((((dnode[1]-snode[1])*ol3[i][0])-((dnode[0]-snode[0])*ol3[i][1])+(dnode[0]*snode[1])-(dnode[1]*snode[0])))/math.sqrt(((dnode[1]-snode[1])**2)+((dnode[0]-snode[0])**2))
        d2=math.sqrt(((snode[0]-ol3[i][0])**2)+((snode[1]-ol3[i][1])**2)) 
        d3=math.sqrt(dist**2+d2**2)
        if((d3<d4)and(dist<ol3[i][2])):
            ol4.append(ol3[i])
    o=len(ol4)
    if(a==1):
        return(o)
    else:
        pobs=ol4[0]
        for i in range (len(ol4)-1):
            d2=math.sqrt(((snode[0]-ol4[i][0])**2)+((snode[1]-ol4[i][1])**2))
            d5=math.sqrt(((snode[0]-ol4[i+1][0])**2)+((snode[1]-ol4[i+1][1])**2))
            if(d5<d2):
                pobs=ol4[i+1]
        px=(dnode[0]-snode[0])
        py=(dnode[1]-snode[1])
        t_theta=math.atan(float(py/px))
        print "thetaaaaa",t_theta  
        print "perendicularr  distance=",dist
        print "robable obstacle==",pobs
        return(pobs,t_theta,dist)   

def two_points(obsnode,t_theta,a):
    print t_theta
    no1=list()
    no2=list()
    for i in range(-a,a+1):       
         node1=list()
         node2=list()
         angle=t_theta+(i*((math.pi)/12))  # to decrease angle between points
         node1.append(int(obsnode[0]-((obsnode[2]+int(2*15/scale_factor))*(math.sin(angle)))))
         node1.append(int(obsnode[1]+((obsnode[2]+int(2*15/scale_factor))*(math.cos(angle)))))
         node2.append(int(obsnode[0]+((obsnode[2]+int(2*15/scale_factor))*(math.sin(angle)))))
         node2.append(int(obsnode[1]-((obsnode[2]+int(2*15/scale_factor))*(math.cos(angle)))))
         no1.append(node1)
         no2.append(node2)
    return no2,no1
def arrange(li,dnode):
    l=len(li)
    df=((li[0][0]-dnode[0])**2)+((li[0][1]-dnode[1])**2)
    dl=((li[l-1][0]-dnode[0])**2)+((li[l-1][1]-dnode[1])**2)
    if(dl>df):
        li.reverse()
    return li
    
def no_add_points(obs,dist):           
    an=math.acos(float(dist/obs[2]))
    ang=(2*an*180)/math.pi
    if ((ang>150)and(ang<=180)):
        return 3
    else:
        an=math.floor(ang/30)# to change the number of points
        return int(an)

def reP(l):
    mid=int(len(l)/2)
    return l[mid]
def dista(m1,m2):
    return ((m1[0]-m2[0])**2+(m1[1]-m2[1])**2) 


def angle(ln,sn,dn):
    asq=dista(dn,ln)
    bsq=dista(dn,sn)
    csq=dista(ln,sn)
    a=math.acos((asq-bsq-csq)/(-2*(math.sqrt(bsq))*(math.sqrt(csq))))
    a=(180*a)/math.pi
    return a


def process(sn,dn,i):
    if (no_of_obstacles(reP(sn),reP(dn),1)==0):
        return(dn)
    else:
        obstacle,t_theta,dist=no_of_obstacles(reP(sn),reP(dn),0)
        add=no_add_points(obstacle,dist)
        n2,n1=two_points(obstacle,t_theta,add)
        ln=wpary2[i-1]
        Pn1=reP(n1)
        Pn2=reP(n2)
        ang=angle(ln,reP(sn),reP(dn))
        ang1=angle(ln,reP(sn),Pn1)
        ang2=angle(ln,reP(sn),Pn2)
        if(ang<90):#degre
            if(ang1>ang2):
                return(process(sn,n1,i))
            else:
                return(process(sn,n2,i))
        else:
            e=no_of_obstacles(reP(n1),reP(dn),1)
            f=no_of_obstacles(reP(sn),reP(n1),1)
            g=no_of_obstacles(reP(n2),reP(dn),1)
            h=no_of_obstacles(reP(sn),reP(n2),1)
            if((e+f)>(g+h)):
                return(process(sn,n2,i))
            elif(((e+f)==(g+h))and(i!=0)):
                if(ang1>ang2):
                    return(process(sn,n1,i))
                else:
                    return(process(sn,n2,i))
            else:
                return(process(sn,n1,i))


        
scale_factor=int(raw_input("Enter scaling factor"))
flist=geofencedata("suas_geo.txt")
latdist,londist,latlim,lonlim,row_s,col_s=rows_and_cols(flist)
img,polyarray=array2image(row_s,col_s,flist)
cv2.imshow("img",img)
cv2.waitKey(0)
ol2=so_obstacle("so_suas.txt")
wpary2,wlist=waypoint("wp_suas.waypoints")
cv2.imshow("img",img)
cv2.waitKey(0)

print("obstacle list")
for i in range(len(ol2)):
  ol2[i][1]=row_s-int(ol2[i][1])
print ol2  

print("wayoint list==============================")  
print(wpary2)  

print("wayoint list")  
for i in range(len(wpary2)):
  wpary2[i][1]=row_s-int(wpary2[i][1])
print(wpary2)

ww=list()
new_waypoint=list()
ww.append(wpary2[0][0])
ww.append(wpary2[0][1])
new_waypoint.append(ww)

print("-----------------------------------------------------------------------")
point=list()
p1=list()

i=0
while (i<(len(wpary2)-1)):
    start_node=list()
    dest_node=list()
    stnode=[wpary2[i][0],wpary2[i][1]]         
    denode=[wpary2[i+1][0],wpary2[i+1][1]]
    start_node.append(stnode)
    dest_node.append(denode)    
    p1=process(start_node,dest_node,i)
    p1=arrange(p1,denode)

    if(reP(p1)!=dest_node[0]):
        for j in range (len(p1)):
             wpary2.insert(i+j+1,p1[j])
        i=i+j+1
    else:
        i=i+1
print "\n\n\n\n\n\n\n"      
print "wParrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr"
print wpary2
print "wParrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr"
waypoint=wpary2
for i in range(len(waypoint)):
     waypoint[i][1]=row_s-int(waypoint[i][1])

print "wayoint list======",waypoint
  
for i in range(len(waypoint)):
        cv2.circle(img,(waypoint[i][0],waypoint[i][1]),5, (100,100,100), -1)
pt = np.array(waypoint, np.int32)
cv2.polylines(img, [pt], False, (0),thickness=1)
    
fnew=open("newwaypoints.txt","w")
fnew.write("QGC WPL 110\n")        
for i in range(len(waypoint)):
     a,b=grid2gps(waypoint[i][1],waypoint[i][0])
     p= str(i)+"    0   3   16  0.000000    0.000000    0.000000    0.000000    "+str(a)+"    "+str(b)+"    60.000000       1"
     fnew.write(p+"\n")
fnew.close()

print img.shape
cv2.imshow("img",img)
cv2.waitKey(0)



