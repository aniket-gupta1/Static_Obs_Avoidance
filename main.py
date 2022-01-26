## input altitudes are in feet
## calculations take place in meters
## input latitudes and longitudes are in decimal format
## all the geometric shapes are plotted in [col][row] format
## all numpy are plotted in [row][col] format

# include altitude in the code
# trimodify once then testaf afterwards





#Prerequisite import
from geopy.point import Point
from geopy import distance
import numpy as np
from scipy import ndimage
import cv2
import math
import time

# returns an list of gps coordinates of geofence points in [lat][lon] 
def geofencedata(Geofence_file):
    fence = open(Geofence_file,"r")
    fencegps=list()
    count=0
    for i in fence.readlines():
        thisline=i.split()
        fencegps.append([float(thisline[0]),float(thisline[1])])
        fence.close()
    return fencegps          

# returns 1. the number of rows and cols of the image matrix depending on scale factor 
#         2. max/min latitude max/min longitude    
def rows_and_cols(fencegps):
    latlimits=[]
    lonlimits=[]
    fencegps=np.asarray(fencegps)                       # converting list to numpy array
    latlimits.append(float(min(fencegps[:,0])-0.001))           
    latlimits.append(float(max(fencegps[:,0])+0.001))
    lonlimits.append(float(min(fencegps[:,1])-0.001))
    lonlimits.append(float(max(fencegps[:,1])+0.001))
    p1 = Point(latlimits[0],lonlimits[0])                      #     p4------------------p3
    p2 = Point(latlimits[0],lonlimits[1])                      #      |                  |
    p3 = Point(latlimits[1],lonlimits[1])                      #      |                  |
    p4 = Point(latlimits[1],lonlimits[0])                      #     p1------------------p2
    d1=(distance.distance(p1,p2).meters)                
    d2=(distance.distance(p3,p4).meters)
    lon_dist=max(d1,d2)                                   # max of the distance between longitudes
    lat_dist=(distance.distance(p3,p2).meters)            # distance between latitudes
    rows=int(lat_dist/scale_factor)                        
    cols=int(lon_dist/scale_factor)
    if (rows%scale_factor != 0):
        rows+=1
    if (cols%scale_factor != 0):
        cols+=1  
    return latlimits,lonlimits,rows,cols   

# swaps the elements of the array row to column and vice versa
def swap(array):
    for i in range(len(array)):
        t=array[i][1]
        array[i][1]=array[i][0]
        array[i][0]=t
    return array    

# converts the given gps coordinates to [row][col] ((((--[row]from top to bottom- --[col] -from left to right--))))) format for an image matrix "OK TESTED"
def gps2grid(lat,lon):
    row=row_s-(row_s*((lat-lat_limits[0])/(lat_limits[1]-lat_limits[0])))
    col=col_s*((lon-lon_limits[0])/(lon_limits[1]-lon_limits[0]))
    return [int(row),int(col)]

# converts the given [row][column] to gps coordinates [lat][lon] 
def grid2gps(row,col):
    lon=((col*(lon_limits[1]-lon_limits[0]))/col_s)+lon_limits[0]
    lat=(((row_s-row)*(lat_limits[1]-lat_limits[0]))/row_s)+lat_limits[0]
    return (lat,lon)

# forms an image with geofence plotted on it  "OK TESTED"
def array2image(rows,cols,fencegps):
    image=np.zeros((rows,cols), dtype=np.uint8)               # image replica of mission plan
    polygonarray=[]                                           # array [row][col], for the image ,of the points of goefence
    for j in range(len(fencegps)):
        polygonarray.append(gps2grid(float(fencegps[j][0]),float(fencegps[j][1])))    
    pts = swap(np.array(polygonarray, np.int32))              # array [col][row] as cv2 prints in (x,y) i.e [row][col] format
    cv2.polylines(image, [pts], True, (255),thickness=1)                                                                                 #############CAN BE SOME IMROVEMENT##########
    cv2.fillPoly(image,[pts],(255))
    return image,polygonarray

#convert feet to meters
def ft2me(x):
    return(x*0.3048)

#convert meters to feet 
def me2ft(x):
    return(x/0.3048)    
    
# to plot obstacles on image         "OK TESTED"
def plot_obstacle(obstacle_file):
    file=open(obstacle_file,"r")
    obstaclelist=list()
    lines = file.readlines()
    for i in lines:
        thisline = i.split()
        row,col=gps2grid(float(thisline[0]),float(thisline[1]))
        l=list()
        l.append(row)                                                 
        l.append(col)
        radius=ft2me(float(thisline[2]))
        if radius<=scale_factor:
            radius=1
        else:
            radius=int(radius/scale_factor)
            if (radius%scale_factor != 0):
                radius+=1
        l.append(radius)
        l.append(math.ceil((ft2me(float(thisline[3])))/scale_factor))
        cv2.circle(image,(col,row), radius, (100,100,100), -1)
        obstaclelist.append(l)
    file.close()
    return(obstaclelist)

# to plot wayoints on image          "OK TESTED"
def plot_waypoint(waypoint_file):
    file= open(waypoint_file,"r")
    waypointgps=list()
    j=0
    lines=file.readlines()
    waypointarray=[]
    for i in lines:
        thisline = i.split()
        if j>=1:
            l=list()
            x=list()
            x.append(float(thisline[8]))
            x.append(float(thisline[9]))
            l=gps2grid(x[0],x[1])
            l.append(math.ceil(ft2me(float(thisline[10]))/scale_factor))
            x.append(math.ceil(ft2me(float(thisline[10]))/scale_factor))
            waypointarray.append(l)
            waypointgps.append(x)
        j+=1
    return waypointarray,waypointgps

def yc(Y):
    return(row_s-Y)

    
def no_of_obstacles(snode,dnode,a): # here the calculations are done in (x,y) coordinate system
    sort_alt=list()
    sort_slope=list()
    sort_distance=list()
    delrow=dnode[0]-snode[0]
    delcol=dnode[1]-snode[1]
    distdest=distance_points(snode,dnode)

    #sort slope
    for i in range(len(obstacle_list)):
        delrow2=obstacle_list[i][0]-snode[0]
        delcol2=obstacle_list[i][1]-snode[1]
        if((delrow2>0)and(delrow<0)):
            continue
        elif((delcol2>0)and(delcol<0)):
            continue
        elif((delrow2<0)and(delrow>0)):
            continue
        elif((delcol2<0)and(delcol>0)):
            continue
        else:
            sort_slope.append(obstacle_list[i])       # obstacle list after slope sorting
    # sort distance       
    for i in range (len(sort_slope)):
        distobs=distance_points(snode,sort_slope[i])
        perpend_dist=distobs*math.sin((angle(sort_slope[i],snode,dnode)/180.0)*math.pi)
        if((distobs<distdest)and(perpend_dist<sort_slope[i][2])):
            sort_distance.append(sort_slope[i])      
    # sort altitude
    for i in range(len(sort_distance)):
        distobs=distance_points(snode,sort_distance[i])
        current_alt=snode[2]+(((dnode[2]-snode[2])*distobs)/distdest)
        if(current_alt<=sort_distance[i][3]):                                   # if altitudes of start node destination  node and obstacles are same obstacle is considered 
            sort_alt.append(sort_distance[i])
    o=len(sort_alt)
    if(a==1):
        return(o)
    else:
        pobs=sort_alt[0]
        for i in range (len(sort_alt)-1):
            d2=distance_points(snode,sort_alt[i])
            d5=distance_points(snode,sort_alt[i+1])
            if(d5<d2):
                pobs=sort_alt[i+1]
        prow=-(dnode[0]-snode[0])
        pcol=(dnode[1]-snode[1])
        t_theta=math.atan(float(prow/pcol)) 
        return(pobs,t_theta,perpend_dist)

# gives the list of the points that are to be plotted around the obstacles 
def two_points(obsnode,t_theta,a,ang):
    no1=list()
    no2=list()
    for i in range(-a,a+1):       
         node1=list()
         node2=list()
         angle=t_theta+(math.pi/2)+(i*((math.pi)/12)) # to decrease angle between points
         node1.append(int(obsnode[0]-((obsnode[2]+int(2*15/scale_factor))*(math.sin(angle)))))
         node1.append(int(obsnode[1]+((obsnode[2]+int(2*15/scale_factor))*(math.cos(angle)))))
         node1.append(obsnode[3])
         node2.append(int(obsnode[0]+((obsnode[2]+int(2*15/scale_factor))*(math.sin(angle)))))
         node2.append(int(obsnode[1]-((obsnode[2]+int(2*15/scale_factor))*(math.cos(angle)))))
         node2.append(obsnode[3])
         no1.append(node1)
         no2.append(node2)
    return no2,no1


# to return the given list in reverse or same order deending on the distance of first and last node from the destination node
def arrange(list1,destnode):
    length=len(list1)
    distfirst=distance_points(list1[0],destnode)
    distlast=distance_points(list1[length-1],destnode)
    if(distlast>distfirst):
        list1.reverse()
    return list1

# returns the altitude list of current given new waypoint list
def current_altitude(list1,snode,dnode):
    s=represent_node(snode)
    d=represent_node(dnode)
    alt_list=list()
    for i in range(len(list1)):
        alt_list.append(s[2]+(((math.cos((math.pi/180.0)*angle(list1[i],s,d)))*distance_points(s,list1[i]))/distance_points(d,s))*(d[2]-s[2]))
    return alt_list

# gives the number of points that are to be added across obstacles 
def no_add_points(obs,dist):
    p=0  # maximum no of points that should be added around the obstacles
    if(obs[2]<=(ft2me(90)/scale_factor)):
        p=0
    elif(obs[2]<=(ft2me(150)/scale_factor)):
        p=0
    elif(obs[2]<=(ft2me(210)/scale_factor)):
        p=1
    else:
        p=2    
    an=math.acos(float(dist/obs[2]))
    ang=(2*an*180)/math.pi             # angle in degrees made by intersection points on center of obstacle
    return 2,ang           # first is the number of points to be plotted around the obsstacle
 
# gives the middle node of the given list
def represent_node(l):
    mid=int(len(l)/2)
    return l[mid]


# gives distance between points p1 and p2
def distance_points(p1,p2):
    return (math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)) 

# gives angle in degrees betwwen lines formed by points 1,2,3 using cosine formula
def angle(lastnode,sourcenode,destnode):
    a_sqr=(distance_points(destnode,lastnode))**2
    b_sqr=(distance_points(destnode,sourcenode))**2
    c_sqr=(distance_points(lastnode,sourcenode))**2
    ang=math.acos((a_sqr-b_sqr-c_sqr)/(-2*(math.sqrt(b_sqr))*(math.sqrt(c_sqr))))
    ang=(180*ang)/math.pi
    return ang

# the main function that handles the path to be choosen 
def process(sn,dn,i1):
    if (no_of_obstacles(represent_node(sn),represent_node(dn),1)==0):
        return(dn)
    else:
        obstacle,t_theta,dist=no_of_obstacles(represent_node(sn),represent_node(dn),0)
        add,ang_le=no_add_points(obstacle,dist)
        n2,n1=two_points(obstacle,t_theta,add,ang_le)
        ln=waypoint_array[i1-1]
        Pn1=represent_node(n1)
        Pn2=represent_node(n2)
        ang=angle(ln,represent_node(sn),represent_node(dn))
        ang1=angle(ln,represent_node(sn),Pn1)
        ang2=angle(ln,represent_node(sn),Pn2)
        if(i1=0):
            ang=91
        if(ang<90):#degre
            if(ang1>ang2):
                return(process(sn,n1,i1))
            else:
                return(process(sn,n2,i1))
        else:
            e=no_of_obstacles(represent_node(n1),represent_node(dn),1)
            f=no_of_obstacles(represent_node(sn),represent_node(n1),1)
            g=no_of_obstacles(represent_node(n2),represent_node(dn),1)
            h=no_of_obstacles(represent_node(sn),represent_node(n2),1)
            if((e+f)>(g+h)):
                return(process(sn,n2,i1))
            elif(((e+f)==(g+h))and(i1!=0)):
                if(ang1>ang2):
                    return(process(sn,n1,i1))
                else:
                    return(process(sn,n2,i1))
            else:
                return(process(sn,n1,i1))
def triangular(startnode,destnode):
    i=0
    waypointarray=list()
    start_node=list()
    dest_node=list()
    start_node.append(startnode)                     # start node is a list containing start [row][col][alt]
    dest_node.append(destnode)                    # destination node is a list containing destination [row][col][alt]   
    new_points=process(start_node,dest_node,i)
    new_points=arrange(new_points,destnode)
    for k in range(len(new_points)):
        new_points_altitude=current_altitude(new_points,start_node,dest_node)
        new_points[k][2]=math.ceil(new_points_altitude[k])
    if(represent_node(new_points)!=dest_node[0]):
        for j in range (len(new_points)):
            waypointarray.append(new_points[j])
    return  waypointarray                      

def mark_attractive(wo):                   # fuction marks the given Point "wo" on the nP array and returns its morPhological function
    imgwo=np.ones((row_s,col_s), dtype=np.uint8)
    imgwo[wo[1]][wo[0]]=0
    edt=ndimage.morphology.distance_transform_edt(imgwo)
    return edt
def sign(number):                  # function returns the sign of the number  
    if(number<0):
        return -1
    elif(number>0):
        return 1 
    else:
        return 0
def flor(x):                       # fuction returns floor value of a float in integer eg -12.10000034  to -13 
    return(int(math.floor(x)))
def yc(Y):
    return(row_s-Y)        
#---------------============= THE MAIN CODE STARTS HERE
scale_factor=int(raw_input("Enter scaling factor"))  # smaller scale factor results in larger image


fence_gps=geofencedata("suas_geo.txt")              # array containing gps coordinates [lat][lon] of geofence


lat_limits,lon_limits,row_s,col_s=rows_and_cols(fence_gps) # row_s are nnumber of rows of image matrix, col_s are number of columns of the image matrix
image,polygon_array=array2image(row_s,col_s,fence_gps)    # image projection of actual mission  # polygon_array is array of geofence points [row][col] format 
#---- image formed with geo fence 

print "rows and cols are",row_s,col_s

obstacle_list=plot_obstacle("so_suas.txt")                 # list of obstacles in [row][column] format
print "obstacles list==",obstacle_list
print "\n"
#-- obstacles plotted

waypoint_array,waypoint_gps=plot_waypoint("wp_suas.waypoints") # list of waypoints in [row][col] or [lat][lon] format respectively
print "waypoint list==",waypoint_array
print"\n\n"

#-- waypoint plotted
cv2.imshow("img",image)
cv2.waitKey(0)

pathlist=list() # will contain the list of waypoints n pixels apart "n is decided in line 388"
#-----------------------------------------------------------------------------------

gradient_img=np.array(image,dtype=np.int8)                         
gradient_img=image/255                                               # converts the image to 0 and 1 with "zero" rePresenting obstacles and geofence boundary and one rePresenting safe Path
rep_gradient=ndimage.morphology.distance_transform_edt(gradient_img)       # ceeates the morPhological image of the rePulsive 

np.set_printoptions(threshold=np.inf,linewidth=np.inf)     # uses the whole terminal to Print the array


i1=0
c=0
l=0
while (i1<(len(waypoint_array)-1)):                                 # wPary 2 is the list of wayPoints in [column][row] format
    start_node1=list(waypoint_array[i1])                                         # ith node as start node
    dest_node1=list(waypoint_array[i1+1])                                 # i+1 th node as destination node

    print ("startnode",start_node1)
    print ("destination node",dest_node1)
    cv2.circle(image,(start_node1[1],start_node1[0]),5, (0,0,0), -1)  
    cv2.circle(image,(dest_node1[1],dest_node1[0]),5, (0,0,0), -1)  

    att_gradient=np.zeros((row_s,col_s))                         
    att_gradient=mark_attractive(dest_node1)                          # edtwo is the morPhological function for the attractive function
        # only for visibility of end node
    cv2.imshow("img",image)
    cv2.waitKey(0)

    dr=(rep_gradient/100.0) + 1
    d0 = 2.0
    nu =400.0
    repulsive=nu*((1/dr-1/d0)**3)
    repulsive=np.where(dr>2.0,0,repulsive)                  # array storing rePulsive values


    xi = 1/700.0
    attractive=att_gradient*2                   # array storing attractive values
    

    net_force = attractive#+repulsive             #array storing net values
 
    fx=(np.array(np.gradient(-net_force,axis=1)))                   # gradient along x axis 
    fy=(np.array(np.gradient(-net_force,axis=0)))                       # graddient along y axis

    array_image=np.array(image,dtype=np.int8)  
    print "#####################################",waypoint_array                      
    while ([flor(start_node1[0]),flor(start_node1[1])]!=[dest_node1[0],dest_node1[1]]):       # works till current node is not equal to dest node
        c+=1
        z=flor(start_node1[0])
        w=flor(start_node1[1])

        delta = [fx[w][z],fy[w][z]]                         # delta stores gradient values of x and y axis
        mag=delta/np.linalg.norm(delta)                     #normalisation of delta values
        
        start_node1[0]=start_node1[0]+mag[0]                              # normalized values gets added to the current node  ( till now all values are in float)
        start_node1[1]=start_node1[1]+mag[1] 
        if((c%30==0)|(c==0)):                     # here 10 in n the spacing between waypoints
            node12=[flor(start_node1[0]),flor(start_node1[1]),start_node1[2]]       
            pathlist.append(node12)
            print pathlist[-1]
            print "==========================",pathlist
            cv2.circle(image,(node12[1],node12[0]),2, (200,200,200), -1)  

        if(array_image[flor(start_node1[0])][flor(start_node1[1])]==100):
            del pathlist[-1]
            del pathlist[-1]
            print "==========================",pathlist
            print "found bruuhhh"
            print len(pathlist)
            node=pathlist[len(pathlist)-1]
            print node
            circle_waypoints=triangular(node,dest_node1)
            circle_waypoints.insert(0,node)
            print circle_waypoints
            print "#####################################",waypoint_array
            for j in range (len(circle_waypoints)):

                waypoint_array.insert(i1+j+1,circle_waypoints[j])
            print "++++++++++++++++++++++++++++",waypoint_array
            break
        cv2.circle(image,(flor(start_node1[1]),flor(start_node1[0])), 0, (175,175,175), -1)    # marks the returned node in the image 
        cv2.imshow("img",image)
        cv2.waitKey(0)
    i1=i1+1                               # net array



print "waypoint list======",waypoint_array

#---------------------------ONLY FOR VISUALIZATION------------------------------------  
for i in range(len(waypoint_array)):
        cv2.circle(image,(waypoint_array[i][1],waypoint_array[i][0]),5, (100,100,100), -1)  
waypointarray=[]
for i in range(len(waypoint_array)):
    waypointarray.append([waypoint_array[i][0],waypoint_array[i][1]])        
pt=np.asarray(swap(waypointarray))
cv2.polylines(image,[pt], False, (0),thickness=1)
cv2.imshow("img",image)
cv2.waitKey(0)
#-------------------------- ONLY FOR VISUALIZATION----------------------------


#---------------------------TO MAKE A NEW NEW WAYPOINT FILE-------------------
file_new=open("newwaypoints.txt","w")
file_new.write("QGC WPL 110\n")        
for i in range(len(waypoint_array)):
     latt,lonn=grid2gps(waypoint_array[i][1],waypoint_array[i][0])
     string = str(i)+"    0   3   16  0.000000    0.000000    0.000000    0.000000    "+str(latt)+"    "+str(lonn)+"    "+str(me2ft(waypoint_array[i][2]))+"       1"
     file_new.write(string+"\n")
file_new.close()




