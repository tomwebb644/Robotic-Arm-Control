#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import colorsys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge #Used to convert from ros string image to matrix image
from geometry_msgs.msg import Pose2D
from time import sleep


bridge = CvBridge()
stop = False
img = None

list_of_shapes = []
crop_list = []
middle_pos_list =[]
position_list = []



print()
print("#########################################")
print(" Now Actively Reading Gazebo Camera Feed")
print("#########################################")
print()


# Runs when new data is recieved from the camera in gazebo
def callback(data): 
    main_code(data)

# Begins node to read camera data and publish coordinates
def readImage():
    global coord_pub
    rospy.init_node('read_image', anonymous=False)  
    rospy.Subscriber("/ur5_simple/camera/image_raw", Image, callback) 
    coord_pub = rospy.Publisher('box_coordinates', Pose2D, queue_size = 10)
    rate = rospy.Rate(1) 
    rospy.spin() # Keep python running the whole while this node is active.

   
def get_coord(mask, colour_name):
    middle_pos_list = []
    if colour_name == "blue":
        name_num = 1
    elif colour_name == "red":
        name_num = 2
    else:
        name_num = 3
    _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE) # Finds contours
    for i, c in enumerate(contours):         # Loop through all the found contours
        position_list.append((c[0,0,0],c[0,0,1]))
        [x,y,w,h] = cv2.boundingRect(c) # Finds coordinates for corners of boxes
        list_of_shapes.append (((x,y), (x+w,y+h)))
        middle_xy = [int(x+(w/2)), int(y+(h/2))]
        middle_pos_list.append(middle_xy)
        x_coord = middle_xy[0]-600 # x pixel offset to middle of arm's base
        y_coord = middle_xy[1] 
        if y_coord <= 715: # Flips coordinates from top to bottom
            y_coord_inv = abs(y_coord-715)
        else:
            y_coord_inv = (0-(y_coord-715)) # negative indicates behind robot
        y_coord_mm = y_coord_inv*1.37 # Transforms pixels to mm
        x_coord_mm = x_coord*1.37 # Transforms pixels to mm
        y_coord_m = y_coord_mm/1000 # Transforms mm to meters
        x_coord_m = x_coord_mm/1000 # Transforms mm to meters
        middle_pos_list[i][0], middle_pos_list[i][1] = y_coord_m, x_coord_m

        coord_pub.publish(x_coord_m, y_coord_m, name_num) # Publishes x, y and assigned colour number
    print(colour_name)
    print(middle_pos_list)
    
    
    return middle_pos_list

def main_code(Image):
    img_unrot = bridge.imgmsg_to_cv2(Image, "bgr8")
    img = cv2.rotate(img_unrot, cv2.ROTATE_90_CLOCKWISE)
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #red colour range
    low_red = np. array([0, 190, 190])
    high_red = np. array([10, 255, 255])
    red_mask = cv2.inRange(hsv_img, low_red, high_red)
    red = cv2.bitwise_and(img, img, mask = red_mask)
    
    #blue colour range
    low_blue = np.array([110, 60, 0])
    high_blue = np.array([121, 255, 255])
    blue_mask = cv2.inRange(hsv_img, low_blue, high_blue)
    blue = cv2.bitwise_and(img, img, mask = blue_mask) 
    
    #pink colour range
    low_pink = np.array([130, 70, 200])
    high_pink = np.array([170, 255, 255]) 
    pink_mask = cv2.inRange(hsv_img, low_pink, high_pink)
    pink = cv2.bitwise_and(img, img, mask = pink_mask)

    #contours and colour detection
    blue_coord_list = get_coord(blue_mask, "blue")
    red_coord_list = get_coord(red_mask, "red")
    pink_coord_list = get_coord(pink_mask, "pink")

    # 4 Signifies that final coordinate has been sent
    coord_pub.publish(0, 0, 4)
        
    sleep(10)
while True:
    Image = readImage()
    




