import time
from geopy.point import Point
from geopy import distance
import numpy as np
from scipy import ndimage
import cv2
import math
import time

img=np.zeros((1000,1000), dtype=np.uint8)
img=img+100
cv2.imshow("image",img)
cv2.waitKey(0)

