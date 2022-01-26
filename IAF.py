import time
from geopy.point import Point
from geopy import distance
import numpy as np
from scipy import ndimage
import cv2
import math
import time

while (True):
	col= 640
	row= 480
	image=np.zeros((row,col), dtype=np.uint8)
	image=image+255
	cv2.circle(image_coy,(snode[1],snode[0]),5, (175,175,175), -1)  





	cv2.rectangle(image,(384,200),(510,128),(0,255,0),-1)
	cv2.imshow("img",image)
	cv2.waitKey(0)
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
    attractive=att_gradient*2                       # array storing attractive values
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