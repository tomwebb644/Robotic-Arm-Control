#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose2D
from time import sleep
init = False
global box_list
box_list = []

def callback(msg):
    global box_list
    picked_colour_list = []
    single_box_coord = (msg.x, msg.y, msg.theta)
    if single_box_coord in box_list:
        print("Already discovered")
    else:
        box_list.append(single_box_coord)
        print ("x coord:",msg.x)
        print ("y coord:",msg.y)
        if msg.theta == 1:
            colour_name = "blue"
        elif msg.theta == 2:
            colour_name = "red"
        elif msg.theta == 3:
            colour_name = "pink"
        elif msg.theta == 4:
            print("End of Coords")
            box_list = box_list[:-1] # removes last value from list (end message)
            box_colour, box_index = pick_box()
            if box_colour == "blue":
                colour_theta = 1
            elif box_colour == "red":
                colour_theta = 2
            else:
                colour_theta = 3
            for a in range(len(box_list)):
                if box_list[a][2] == colour_theta:
                    curr_coords = (box_list[a][0], box_list[a][1])
                    picked_colour_list.append(curr_coords)
            picked_box_coords = picked_colour_list[box_index]
            print(picked_box_coords)

            
            #function to pick up box from coords
            pick_up_box(picked_box_coords)
    
def pick_up_box(coords):
    print(str(coords[0]),str(coords[1]))
    sleep(5)
    pose_goal.position.x = coords[0]
    pose_goal.position.y = coords[1]
    pose_goal.position.z = 0.1
    group.set_pose_target(pose_goal)
    ## Now, we call the planner to compute the plan and execute it.
    group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()

    sleep(2)



def pick_box():
    print ("What colour would you like to pick up?")
    colour_picked = False
    while colour_picked == False:
        colour_to_pick = raw_input("blue, red or pink?")
        if colour_to_pick == "blue":
            colour_picked = True
        elif colour_to_pick == "red":
            colour_picked = True
        elif colour_to_pick == "pink":
            colour_picked = True
        else:
            print ("Invalid Input! Please choose an option below:")
    print("Would you like", colour_to_pick, "box 1 or 2:")
    index_picked = False
    while index_picked == False:
        index_to_pick = int(raw_input(""))
        if index_to_pick == 1:
            index_picked = True
        elif index_to_pick == 2:
            index_picked = True
        else:
            print ("Invalid Input! Please choose an option below:")
        index_to_pick -= 1
    return colour_to_pick, index_to_pick

def main():
    rospy.Subscriber("/box_coordinates",Pose2D,callback)
    rospy.spin()
    

if __name__ == '__main__':
    if init == False:
        global box_list
        box_list = []
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
        robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
        group_name = 'manipulator'   # default is "manipulator"
        group = moveit_commander.MoveGroupCommander(group_name)
        group.set_num_planning_attempts(5)
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

        print("========== JOINT GOAL: ===========")
        sleep(1)

        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 1.57
        joint_goal[1] = -1.8
        joint_goal[2] = 2
        joint_goal[3] = -1.8
        joint_goal[4] = -1.57
        joint_goal[5] = -1.57

        group.go(joint_goal, wait=True)
        group.stop()

        print("========== POSE GOAL: ===========")
        sleep(1)

        pose_goal = geometry_msgs.msg.Pose()		#gets a blank pose message: pos.x,y,z; orient.x,y,z,w - world reference
        print("========== Pose direct from message: ===========")
        print(pose_goal)
        pose_goal = group.get_current_pose().pose
        print("========== Pose as pulled from current group: ===========")
        print(pose_goal)
#pose_goal.orientation.w = 0 not touching this just yet
        pose_goal.position.x = 0
        pose_goal.position.y = 0.2
        pose_goal.position.z = 0.3
        group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
        group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
        group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

        sleep(2)

        print("initialized")
        init = True
    main()



