## input altitudes are in feet
## calculations take place in meters
## input latitudes and longitudes are in decimal format
## all the geometric shapes are plotted in [col][row] format
## all numpy are plotted in [row][col] format




# include altitude in the code
# trimodify once then testaf afterwards
# to check which wapoint to take when startingh


# THIS PROGRAM IS COMPLETE##############################33


#Prerequisite import
import time
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
    lon_dist=max(d1,d2)                                  # max of the distance between longitudes
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
    imagecoy=np.zeros((rows,cols), dtype=np.uint8)
    image=np.zeros((rows,cols), dtype=np.uint8)               # image replica of mission plan
    polygonarray=[]                                           # array [row][col], for the image ,of the points of goefence
    for j in range(len(fencegps)):
        polygonarray.append(gps2grid(float(fencegps[j][0]),float(fencegps[j][1])))    
    pts = swap(np.array(polygonarray, np.int32))              # array [col][row] as cv2 prints in (x,y) i.e [row][col] format
    cv2.polylines(imagecoy, [pts], True, (255),thickness=1)
    cv2.polylines(image, [pts], True, (255),thickness=1)                                                                                 #############CAN BE SOME IMROVEMENT##########
    cv2.fillPoly(image,[pts],(255))
    cv2.fillPoly(imagecoy,[pts],(255))
    return image,polygonarray,imagecoy

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
        cv2.circle(image_coy,(col,row), radius, (100,100,100), -1)
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

#-----------------------------------obstacle plotting-----------------------------------------------------------


def no_of_obstacles(snode,dnode,pn):      # form [row][col][alt]
    cv2.circle(image_coy,(snode[1],snode[0]),5, (175,175,175), -1)  
    cv2.waitKey(0)
    sort_alt=list()
    sort_slope=list()
    sort_distance=list()
    delrow=dnode[0]-snode[0]
    delcol=dnode[1]-snode[1]
    distdest=distance_points(snode,dnode)
    t_theta=0.0

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
        if((distobs<distdest)and(perpend_dist<(sort_slope[i][2]+10))):
            sort_distance.append(sort_slope[i])      
    # sort altitude
    for i in range(len(sort_distance)):
        distobs=distance_points(snode,sort_distance[i])
        current_alt=snode[2]+(((dnode[2]-snode[2])*distobs)/distdest)
        if(current_alt<=sort_distance[i][3]):                                   # if altitudes of start node destination  node and obstacles are same obstacle is considered 
            sort_alt.append(sort_distance[i])
    pobs=sort_alt[0]
    for i in range (len(sort_alt)-1):
        d2=distance_points(snode,sort_alt[i])
        d5=distance_points(snode,sort_alt[i+1])
        if(d5<d2):
            pobs=sort_alt[i+1]
    prow=-(dnode[0]-snode[0])
    pcol=(dnode[1]-snode[1])
    t_theta=math.atan(float(float(prow)/float(pcol))) 
    return(pobs,t_theta,perpend_dist)


# gives the list of the points that are to be plotted around the obstacles 
def two_points(obsnode,t_theta,a):
    no1=list()
    no2=list()
    for i in range(-a,a+1):
        angle=0.0      
        node1=list()
        node2=list()
        angle=t_theta+(math.pi/2)+(i*((math.pi)/6)) # to decrease angle between points
        node1.append(int(obsnode[0]-((obsnode[2]+int(3*6/scale_factor))*(math.sin(angle)))))
        node1.append(int(obsnode[1]+((obsnode[2]+int(3*6/scale_factor))*(math.cos(angle)))))
        node1.append(obsnode[3])
        node2.append(int(obsnode[0]+((obsnode[2]+int(3*6/scale_factor))*(math.sin(angle)))))
        node2.append(int(obsnode[1]-((obsnode[2]+int(3*6/scale_factor))*(math.cos(angle)))))
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
def current_altitude(list1,s,d):
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
        p=1  
    #an=math.acos(float(dist/obs[2]))
    #ang=(2*an*180)/math.pi             # angle in degrees made by intersection points on center of obstacle
    return p         # first is the number of points to be plotted around the obstacle
 
# gives the middle node of the given list
def represent_node(l):
    mid=int(len(l)/2)
    return l[mid]

# the main function that handles the path to be choosen 
def process(sn,dn,pn):                                        # form [row][col][alt]
        obstacle,t_theta1,dist=no_of_obstacles(sn,dn,pn)
        add=no_add_points(obstacle,dist)  #  to add angle
        n1,n2=two_points(obstacle,t_theta1,add)
        n1=arrange(n1,dn)
        n2=arrange(n2,dn) 
        """for k in range(len(n1)):
            cv2.circle(image_coy,(n1[k][1],n1[k][0]), 2, (0,0,0), -1)
            cv2.circle(image_coy,(n2[k][1],n2[k][0]), 2, (0,0,0), -1)
        cv2.imshow("image",image_coy)
        cv2.waitKey(0) """   
        return n1,n2


# gives distanc between points p1 and p2
def distance_points(p1,p2):
    return (math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)) 

# gives angle in degrees betwwen lines formed by points 1,2,3 using cosine formula
def angle(lastnode,sourcenode,destnode):
    a_sqr=(distance_points(destnode,lastnode))**2
    b_sqr=(distance_points(destnode,sourcenode))**2
    c_sqr=(distance_points(lastnode,sourcenode))**2
    d=(a_sqr-b_sqr-c_sqr)/(2*(math.sqrt(b_sqr))*(math.sqrt(c_sqr)))
    if (int(d)==1):
        d=1.0
    ang=math.acos(d)
    ang=(180.0*ang)/math.pi
    #print ang
    return ang


def flor(x):                       # fuction returns floor value of a float in integer eg -12.10000034  to -13 
    return(int(math.floor(x)))

def triangular(startnode,destnode,point):    # form [row][col][alt]   RETURNS A LIST CONTAINIING TWO SETS OF WAYPOINTS AROUND OBSTACLES PAO[0], PAO[1]
    points_around_obstacle=list()
    new_points1,new_points2=process(startnode,destnode,point)
    for k in range(len(new_points1)):
        new_points_altitude1=current_altitude(new_points1,startnode,destnode)
        new_points1[k][2]=math.ceil(new_points_altitude1[k])
    for k in range(len(new_points2)):
        new_points_altitude2=current_altitude(new_points2,startnode,destnode)
        new_points2[k][2]=math.ceil(new_points_altitude2[k])  
    points_around_obstacle.append(new_points1)
    points_around_obstacle.append(new_points2)
    return points_around_obstacle           

def sign(number):# function returns the sign of the number  
    if(number<0):
        return -1
    elif(number>0):
        return 1 
    else:
        return 0

def yc(Y):
    return(row_s-Y) 


def mark_attractive(wo):                   # fuction marks the given Point "wo" on the nP array and returns its morPhological function
    imgwo=np.ones((row_s,col_s), dtype=np.uint8)
    imgwo[wo[0]][wo[1]]=0
    edt=ndimage.morphology.distance_transform_edt(imgwo)
    return edt 

def apf(node1,node2,listlen1):
    c=1  
    plist=list()
    plist.append([flor(node1[0]),flor(node1[1]),node1[2]])
    cv2.circle(image_coy,(node1[1],node1[0]),2, (0,0,0), -1)  
    cv2.circle(image_coy,(node2[1],node2[0]),2, (0,0,0), -1)  
    att_gradient=np.zeros((row_s,col_s))                         
    att_gradient=mark_attractive(node2)                          # edtwo is the morPhological function for the attractive function
    dr=(rep_gradient/100.0) + 1
    d0 = 2.0
    nu =400.0
    repulsive=nu*((1/dr-1/d0)**3)
    repulsive=np.where(dr>2.0,0,repulsive)                  # array storing rePulsive values
    xi = 1/700.0
    attractive=att_gradient*2.5                        # array storing attractive values
    net_force = attractive+repulsive             #array storing net values
 
    fx=(np.array(np.gradient(-net_force,axis=1)))                   # gradient along x axis 
    fy=(np.array(np.gradient(-net_force,axis=0)))                       # graddient along y axis

    array_image=np.array(image,dtype=np.int8)   

    while ([flor(node1[0]),flor(node1[1])]!=[flor(node2[0]),flor(node2[1])]):       # works till current node is not equal to dest node
        z=flor(node1[0])
        w=flor(node1[1])

        delta = [fy[z][w],fx[z][w]]                         # delta stores gradient values of x and y axis
        mag=delta/np.linalg.norm(delta)                     #normalisation of delta values
        
        node1[0]=node1[0]+mag[0]                              # normalized values gets added to the current node  ( till now all values are in float)
        node1[1]=node1[1]+mag[1] 
        if(c%30==0):             # here 20 in n the spacing between waypoints

            node12=[flor(node1[0]),flor(node1[1]),node1[2]]    
            plist.append(node12)
        
        if(array_image[flor(node1[0])][flor(node1[1])]==100):
            if (len(plist)>1):
                del plist[-1]
            if (len(plist)>2):
                del plist[-1] 

            circle_waypoints=triangular(plist[-1],node2,[flor(node1[0]),flor(node1[1])])    # plist is of the form [row ][coll][alt]
            
            for i in range(len(circle_waypoints[0])):
                cv2.circle(image_coy,(circle_waypoints[0][i][1],circle_waypoints[0][i][0]), 2, (0,0,0), -1)
                cv2.circle(image_coy,(circle_waypoints[1][i][1],circle_waypoints[1][i][0]), 2, (0,0,0), -1)
                cv2.imshow("image",image_coy)
            #cv2.waitKey(0)
            return(join(plist,circle_waypoints,node2),0)
        c+=1    
        cv2.circle(image_coy,(flor(node1[1]),flor(node1[0])), 0, (175,175,175), -1)    # marks the returned node in the image 
        cv2.imshow("image",image_coy) 
    plist.append(node2)            
    return(plist,1)

def join(list1,wp,no2):
    path2=list()
    pathlist1=list(list1)
    pathlist2=list(list1)
    for i in range(len(wp[0])):
        pathlist1.append(wp[0][i])
        pathlist2.append(wp[1][i])   
    pathlist1.append(no2)
    pathlist2.append(no2)   
    path2.append(pathlist1)
    path2.append(pathlist2)
    return(path2)

def list_handle(l1):
    pathlist=list()
    p=list()
    w1=list(l1[len(l1)-2])
    w2=list(l1[len(l1)-1])
    #print "sending spf", w1,w2
    listlen=len(l1)
    pathlist,n=apf(w1,w2,listlen)
    #print "pathlist recieved",pathlist


    if (n==0):
        p.append(list(l1))
        p.append(list(l1))
        if (len(l1)==2):
            return pathlist,n
        else:         
            del p[1][-1]
            del p[1][-1]
            del p[0][-1]
            del p[0][-1]
            for i in range(len(pathlist[0])):
                p[0].append(pathlist[0][i])
                p[1].append(pathlist[1][i])    
    else:
        if (len(l1)==2):
            p.append(pathlist)
            return p,n
        else:   
            p.append(list(l1))
            #print "8888888888888888888",p
            del p[0][-1]
            del p[0][-1]
            #print "8888888888888888888",p

            for i in range(len(pathlist)):
                p[0].append(pathlist[i])
    return p,n


def all_paths(lists):
    allpaths=list()
    length=0
    allpaths.append(lists)
    while(length<len(allpaths)):
        m=0
        #print "length is ", length
        #print "length all paths is",len(allpaths)
        templist=list()
        #print "all paths ",allpaths


        #print "sending list handle",allpaths[length] ,"\n\n"  
        templist,m=list_handle(allpaths[length])
        #print "recieved list handle",templist,"   ",m,"\n\n"
    
        del allpaths[length]
        for k in range (len(templist)):
            allpaths.insert(length+k,templist[k])
        length=length+m    
    return allpaths

def tolist(e1,e2):
    l=list()
    l.append(e1)
    l.append(e2)
    return l  

def poss_path(list1):
    #print "##",list1
    #print "best path entered"
    j=0
    #print "fence interdecton working"


    for i in range (len(list1)):
        if (len(list1)==1):
            return list1   
        retrn=filter_fence_intersection(list1[i-j])
        #print "returned value for ",i,"is",retrn
        if(retrn==0):
            #print "deleted"
            del list1[i-j]
            j+=1
   
    j=0
    #print "impossible turn working"
    for i in range (len (list1)):
        if (len(list1)==1):
            return list1
        if(filter_impossible_turn(list1[i-j])==0):
            #print "deleted"
            del list1[i-j]
            j+=1
    
    return list1
           

def best_path(list1,list2):
    angle_array=list()
    sumangle_array=list()
    sumangle_array2=list()
    count=0
    index_list=list()
    for i in range (len (list1)):
        list21=list1[i]
        l=len(list21)
        angg=angle(list21[l-2],list21[l-1],list2[0][1])
        angle_array.append(angg)
        if (angg<100):
            count+=1
            index_list.append(i)
    i=0
    for i in range (len(list1)):
        sumangle_array.append(filter_min_angle(list1[i]))
        
    if(count==0):
        return list1[sumangle_array.index(min(sumangle_array))]
    else:
        for i in range(len(index_list)):
            sumangle_array2.append(sumangle_array[index_list[i]])
        return list1[index_list[sumangle_array.index(min(sumangle_array2))]]
        
def filter_fence_intersection(listt):
    #print "fence interdecton entered"
    #testimage=image.copy()
    #cv2.imshow("image on test",image)
    #print "####",listt
    for i in range(len(listt)-1):
        node1=list(listt[i])
        node2=list(listt[i+1])
        morph=mark_attractive(node2)
        while ([flor(node1[0]),flor(node1[1])]!=[flor(node2[0]),flor(node2[1])]):       # works till current node is not equal to dest node
            z=flor(node1[0])
            w=flor(node1[1])
            fx=(np.array(np.gradient(-morph,axis=1)))                   # gradient along x axis 
            fy=(np.array(np.gradient(-morph,axis=0)))                       # graddient along y axis


            delta = [fy[z][w],fx[z][w]]                         # delta stores gradient values of x and y axis
            mag=delta/np.linalg.norm(delta)                     #normalisation of delta values
        
            node1[0]=node1[0]+mag[0]                              # normalized values gets added to the current node  ( till now all values are in float)
            node1[1]=node1[1]+mag[1] 
            #cv2.circle(testimage,(flor(node1[1]),flor(node1[0])),0, (100,100,100), -1)
            #cv2.imshow("image22",testimage)

            if((image[flor(node1[0])][flor(node1[1])]==0)|(image[flor(node1[0])][flor(node1[1])]==100)):             # here 20 in n the spacing between waypoints
                return 0
    #cv2.waitKey(0)
    return 1            
        

def filter_min_angle(listt):
    sumangle=0
    for i in range(len(listt)-2):
        ang=angle(listt[i],listt[i+1],listt[i+2])

        sumangle=sumangle+ang    
    return sumangle    

def filter_impossible_turn(listt):
    #for i in range(len(listt)):
        #cv2.circle(image,(listt[i][1],listt[i][0]),2, (0,0,0), -1)
    #cv2.imshow("im2",image) 
    for i in range(len(listt)-2):
        a=angle(listt[i],listt[i+1],listt[i+2])
        if(a>60):
            return 0
    return 1    

def remove_points_near_dest(list1):  # to remive points that are very near to the destionation point
    #print "entered remove points"
    """
    cv2.waitKey(0)
    testimage=image.copy() 
    for i in range(len(list1)):
        for j in range(len(list1[i])):
            cv2.circle(testimage,(list1[i][j][1],list1[i][j][0]),1, (200-(50*i),200-(50*i),200-(50*i)), -1)
        cv2.imshow("image23",testimage)
        cv2.waitKey(0)
    """
    for i in range(len(list1)):
        list12=list1[i]
        l1=len(list12)
        if(l1>2):
            d=distance_points(list12[l1-1],list12[l1-2])
            if (d<15):
                del list12[l1-2]
                #print "removed one point near destination from path ",i
    return list1  

def remove_straight_points(listt):
    j=0
    for i in range(1,len(listt)-1):
        ang=angle(listt[i-j-1],listt[i-j],listt[i-j+1])
        if (ang<3):
            del listt[i-j]
            j+=1 
    return listt 





#---------------============= THE MAIN CODE STARTS HERE
scale_factor=int(raw_input("Enter scaling factor"))  # smaller scale factor results in larger image

start_time = time.clock()

fence_gps=geofencedata("suas_geo.txt")              # array containing gps coordinates [lat][lon] of geofence


lat_limits,lon_limits,row_s,col_s=rows_and_cols(fence_gps) # row_s are nnumber of rows of image matrix, col_s are number of columns of the image matrix
image,polygon_array,image_coy=array2image(row_s,col_s,fence_gps)    # image projection of actual mission  # polygon_array is array of geofence points [row][col] format 
#---- image formed with geo fence 
np.set_printoptions(threshold=np.inf,linewidth=np.inf)
image_np=np.array(image,dtype=np.int8)   # FORMS NUMpY ARRAY of 0(black) and -1(white)                 
image_rep=image/255 
#cv2.imshow("yup",image)
#cv2.waitKey(0)                                                      # converts the image to 0 and 1 with "zero" rePresenting obstacles and geofence boundary and one rePresenting safe Path
rep_gradient=ndimage.morphology.distance_transform_edt(image_rep)


obstacle_list=plot_obstacle("so_suas.txt")                 # list of obstacles in [row][column] format
print "obstacles list  =",obstacle_list
print "\n"
#-- obstacles plotted

waypoint_array,waypoint_gps=plot_waypoint("wp_suas.waypoints") # list of waypoints in [row][col] or [lat][lon] format respectively
print "waypoint list  =",waypoint_array


#-- waypoint plotted
cv2.imshow("image",image_coy)
cv2.waitKey(0)


print "----------------------IMAGE FORMED----------------------------"
#-----------------------------------------------------------------------------------
i1=0
final_path=list()   
all_possible_paths_list=list()   
while (i1<(len(waypoint_array)-1)):                       # waypoint_array is the list of wayPoints in [row][col] format
    #print "pair points ",i1
    all_possible_paths=list()                                # contains list of all possible paths
    selected_path=list()
                                         # i+1 th node as destination node   

    print i1                                                                  
    all_possible_paths=all_paths(tolist(waypoint_array[i1],waypoint_array[i1+1]))         # return all_pathlist will return a list containing all paths

    all_possible_paths_list.append(all_possible_paths)
    i1=i1+1


i=0
while (i<(len(all_possible_paths_list))):         

    all_possible_paths_list[i]=remove_points_near_dest(all_possible_paths_list[i])
    
    all_possible_paths_list[i]=poss_path(all_possible_paths_list[i])  # step is taking most time

    

    i=i+1
i=0

"""
for i in range(len(all_possible_paths_list)):
    print "################$$$$$$$$$$$$$",all_possible_paths_list[i],"\n\n"
    cv2.waitKey(0)
"""





"""
print "all valid possible paths reached"
cv2.waitKey(0)
print all_possible_paths_list
#print "\n\n\n\n\n\n\n"
"""


i=0

while(i<len(all_possible_paths_list)):
   
    if (i==(len(all_possible_paths_list)-1)):
        angle_array2=list()
        for j in range(len(all_possible_paths_list[i])):
            print filter_min_angle(all_possible_paths_list[i][j])
            angle_array2.append(filter_min_angle(all_possible_paths_list[i][j]))
        selected_path=all_possible_paths_list[i][angle_array2.index(min(angle_array2))]


    else:

        
        selected_path=best_path(all_possible_paths_list[i],all_possible_paths_list[i+1])
        #print selected_path
        """

        testimage2=image.copy()
        for j in range(len(selected_path)):
            cv2.circle(testimage2,(selected_path[j][1],selected_path[j][0]),2, (0,0,0), -1)
        cv2.imshow("imgmain",testimage2)
        print "jingalala huhuhuhuhuhuhuhuhuhuhuhuhuhuh" 
        cv2.waitKey(0)\
        """

    selected_path=remove_straight_points(selected_path)

    for k in range(1,len(selected_path)):
        final_path.append(selected_path[k])
        #selected_path=select(all_possible_paths)               # contains selected path's waypoint list excluding starting point and end point 
        #final_path=reduce_waypoints(selected_path)             # filter outs some waypoints 
        #for j1 in range(len(final_path)):
        #    waypoint_array.insert(i1+j1+1,final_path[j1])
    i=i+1

i=0


print final_path    

testimage3=image.copy()
for i in range(len(final_path)):

    cv2.circle(testimage3,(final_path[i][1],final_path[i][0]),2, (0,0,0), -1)
cv2.imshow("im123432432",testimage3) 
cv2.waitKey(0)      
print "reached finally"
print time.clock() - start_time, "seconds"  
pt=list()

for i in range(len(final_path)):
    point1=[final_path[i][1],final_path[i][0]]
    pt.append(point1)

pt=np.asarray(pt)
cv2.polylines(testimage3,[pt],True,(100),thickness=1)
cv2.imshow("imagggg",testimage3)
cv2.waitKey(0)

"""                                     


                     # net arra

#---------------------------ONLY FOR VISUALIZATION------------------------------------  
for i in range(len(pathlist)):
        cv2.circle(image,(pathlist[i][1],pathlist[i][0]),2, (0,0,0), -1)  
patharray=[]
for i in range(len(pathlist)):
    patharray.append([pathlist[i][0],pathlist[i][1]])        
pt=np.asarray(swap(patharray))
cv2.polylines(image,[pt], False,(0),thickness=1)
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
file_new.close()a
"""