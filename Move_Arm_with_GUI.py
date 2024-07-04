#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import cv2
from math import pi
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Used to convert from ros string image to matrix image
from time import sleep
init = False
first_msg = True
bridge = CvBridge()

# callback for when an image is recieved, used for the GUI
def callback_image(msg):
    global first_msg 
    if first_msg == True: #only runs the first time an image message is recieved
        print("Image recieved")
        global img
        img_unrot = bridge.imgmsg_to_cv2(msg, "bgr8") # converts ros image to opencv image
        img = cv2.rotate(img_unrot, cv2.ROTATE_90_CLOCKWISE) # rotates image so it is easier to see
        first_msg = False

# callback for when an coordinates are recieved
def callback(msg):
    global box_list
    picked_colour_list = []
    single_box_coord = (msg.x, msg.y, msg.theta) # pulls coordinates out of ros message
    if single_box_coord in box_list: # prevent's duplicate coordinates
        pass
    else:
        box_list.append(single_box_coord) # adds new coordinates to a list
        if msg.theta == 4: # signifies end of coordinates being published

            box_list = box_list[:-1] # removes last value from list (end message)
            # Launches GUI to select box
            clicked_coords = pick_box()
            # Finds box nearest user input
            picked_box_x_coord, picked_box_y_coord = find_closest_box(box_list, clicked_coords)
            #print(picked_box_x_coord, picked_box_y_coord)

            # function to pick up box from coords
            pick_up_box(picked_box_x_coord, picked_box_y_coord)


# Detects clicks inside image window
def click_event(event, x, y, flags, params): 
    global clicked_coords
    # checking for left mouse clicks 
    if event == cv2.EVENT_LBUTTONDOWN: 
         clicked_coords = (x, y)

    # checking for right mouse clicks      
    if event==cv2.EVENT_RBUTTONDOWN: 
        clicked_coords = (x, y)

# Sends position of selected box to arm controller
def pick_up_box(x_coord, y_coord):
    sleep(2)
    pose_goal.position.x = x_coord
    pose_goal.position.y = y_coord
    pose_goal.position.z = 0.2
    group.set_pose_target(pose_goal)
    ## Now, we call the planner to compute the plan and execute it.
    group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    group.clear_pose_targets() # Good practice
    sleep(0.5)

    pose_goal.position.z = 0.07
    group.set_pose_target(pose_goal)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    sleep(2) # wait for "gripping"

    pose_goal.position.z = 0.2
    group.set_pose_target(pose_goal)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    sleep(0.5)


    pose_goal.position.x = 0
    pose_goal.position.y = 0.2
    pose_goal.position.z = 0.3
    group.set_pose_target(pose_goal)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    sleep(0.5)

# Finds box nearest user input
def find_closest_box(box_list, clicked_coords):
    x_coord = clicked_coords[0]-600 # x pixel offset to middle of arm's base
    y_coord = clicked_coords[1] 
    if y_coord <= 715: # Flips coordinates from top to bottom
        y_coord_inv = abs(y_coord-715)
    else:
        y_coord_inv = (0-(y_coord-715)) # negative indicates behind robot
    y_coord_m = y_coord_inv*1.37/1000 # Transforms pixels to meters
    x_coord_m = x_coord*1.37/1000 # Transforms pixels to meters
    clicked_coords_m = (x_coord_m, y_coord_m)

    min_distance = 10 # larger than any distance in reach
    # Finds shortest distance between user click and box centers
    for i in range(len(box_list)):
        distance = ((((box_list[i][0]-clicked_coords_m[0])**2)+((box_list[i][1]-clicked_coords_m[1])**2))**0.5) # Pythagoras to find the shortast distance between box centres and mouseclick
        if distance < min_distance: # Replaces lowest value with the next lowest
            min_distance = distance
            picked_box_x_coord, picked_box_y_coord = box_list[i][0], box_list[i][1]


    return picked_box_x_coord, picked_box_y_coord
     
# Launches GUI to select box       
def pick_box():
    global img
    print("Please click on box in the image you would like to pick up.")
    print("Once you have selected the box, press any key to close image window")
    # displays the image 
    cv2.imshow('image', img) 
  
    # setting mouse hadler for the image and calling the click_event() function 
    cv2.setMouseCallback('image', click_event) 
    # wait for a key to be pressed to exit 
    cv2.waitKey(0) 
    # close the window 
    cv2.destroyAllWindows() 
    print("")
    print("Box Selected!")
    return clicked_coords

# Necessary for rospy.spin()
def main():
    rospy.Subscriber("/box_coordinates",Pose2D,callback) # Initialize subscriber
    rospy.Subscriber("/ur5_simple/camera/image_raw", Image, callback_image)
    rospy.spin()
    

if __name__ == '__main__':
    if init == False: # Runs once
        global box_list
        box_list = []
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

        # Instantiate a `RobotCommander`_ object. This object is the outer-level interface to the robot:
        robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object.  This object is an interface to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface to one group of joints.
        group_name = 'manipulator'
        group = moveit_commander.MoveGroupCommander(group_name)
        group.set_num_planning_attempts(5) # Sets number of plan attempts that can be made before failing

#========== GROUND GENERATOR FROM INTERPRETER SOURCE CODE =============#

        pose = geometry_msgs.msg.PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        # offset such that the box is 0.1 mm below ground (to prevent collision with the robot itself)
        pose.pose.position.z = -0.055
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = robot.get_root_link()
        scene.attach_box(robot.get_root_link(), "ground", pose, (3, 3, 0.1), touch_links=[robot.get_root_link()])

#========== GROUND GENERATOR FROM INTERPRETER SOURCE CODE =============#

        sleep(1)
        
        # Sets a joint goal instead of a pose goal to prevent unexpected behaviour while initializing
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 1.57
        joint_goal[1] = -1.8
        joint_goal[2] = 2
        joint_goal[3] = -1.8
        joint_goal[4] = -1.57
        joint_goal[5] = -1.57

        # Calls the planner to compute the plan and execute it.
        group.go(joint_goal, wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()

        sleep(1)

        # Set pose goal and exacuting to move arm out of frame to prevent blocking boxes from the camera's view
        pose_goal = geometry_msgs.msg.Pose()		#gets a blank pose message: pos.x,y,z; orient.x,y,z,w - world reference
        pose_goal = group.get_current_pose().pose
        pose_goal.position.x = 0
        pose_goal.position.y = 0.2
        pose_goal.position.z = 0.3
        group.set_pose_target(pose_goal)

        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

        sleep(1)

        print("Initialized")
        init = True
    main()



