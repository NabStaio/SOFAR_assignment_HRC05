#!/usr/bin/env python 

############################################################################################
#_SCRIPT FOR MOVING THE RIGHT ARM:                                                         #
# This script is needed in order to pick the two blue boxes on the right side of the table #
# and put them on the middle whereby left arm can reach that position and pick the cubes   #
############################################################################################



############################################_LIBRARIES_######################################
from __future__ import print_function

import rospy
import sys
import copy
import time
import moveit_commander 
import moveit_msgs.msg
import geometry_msgs.msg 
import trajectory_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, RobotTrajectory
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from human_baxter_collaboration.msg import BaxterTrajectory, UnityTf

#############################################################################################

global my_data

#callback for subscriber to /unity_tf that it needs for position of cubes and middleplacementN
def callback(data):
    global my_data
    my_data = data.frames
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.frames)

#####################################################################################################
#_FUNCTION MOVE_RIGHT(CUBE):                                                                        #
# This function has as input the cube to pick up, then it plans four steps and publish              #
# them as an array of trajectory to the Unity simulation (moveit_msgs/RobotTrajectory[] trajectory) #
#####################################################################################################

def move_right(cube):
    global my_data, pub

    frame = cube

    
    move_groupr = moveit_commander.MoveGroupCommander('right_arm')
    #first field of baxter_msg to fill, the arm (string arm) 
    baxter_msg = BaxterTrajectory()
    baxter_msg.arm = 'right'


    #setting of the initial state of the robot, all names and positions are on topic /baxter_joint_state or in /joint_state
    #it used a listener to these topics to set them manually
    joint_state = JointState()
    joint_state.name = ['head_pan', 'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2', 'left_s0', 'left_s1', 'left_e0', 'left_e1', 
    'left_w0', 'left_w1', 'left_w2', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint']
    joint_state.position = [0.0, 0.5235987755982988, -1.2217304763960306, 0.0, 1.7278759594743864, 0.0, 0.7504915783575616, 0.0, -0.5235987755982988, 
    -1.2217304763960306, 0.0, 1.7278759594743864, 0.0, 0.7504915783575616, 0.0, 0.0, 0.0, 0.0, 0.0]
    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = joint_state
    move_groupr.set_start_state(moveit_robot_state)
    
    #####################################################################################################
    # NOW GO FOR PLANNING STEPS:                                                                        #
    # 1ST STEP: Go over the cube, so increase a little z position                                       #
    # 2ND STEP: Pick the cube going down                                                                #
    # 3RD STEP: Pick up the object going up, so again increase a little z position                      #
    # 4TH STEP: Place the cube on the middle of the table                                               #
    #                                                                                                   #
    # For each step it needs the positions of all arm's joints, in order to do that it is used the      #
    # variable pose_goal a Pose one to set the pose target, then it is possible to do a plan (plan())   #
    # and execute it (execute(plan[1])), after that it needs to update the position so it is            #
    # reused the set_start_state() function                                                             #
    #####################################################################################################
    
    #################_FIRST STEP GO OVER THE CUBE_#################
    #print("cube position: %s", my_data[frame])
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = my_data[frame].pose.orientation.x
    pose_goal.orientation.y = my_data[frame].pose.orientation.y
    pose_goal.orientation.z = my_data[frame].pose.orientation.z
    pose_goal.orientation.w = my_data[frame].pose.orientation.w
    pose_goal.position.x = my_data[frame].pose.position.x  
    pose_goal.position.y = my_data[frame].pose.position.y 
    pose_goal.position.z = my_data[frame].pose.position.z + 0.2

    move_groupr.set_pose_target(pose_goal)
    plan_overcube = move_groupr.plan()
    move_groupr.execute(plan_overcube[1])

    joint_state = JointState()
    joint_state.name = move_groupr.get_active_joints()
    joint_state.position = move_groupr.get_current_joint_values()
    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = joint_state
    move_groupr.set_start_state(moveit_robot_state)
    ################################################################

    #################_SECOND STEP PICK THE CUBE_#################
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = my_data[frame].pose.orientation.x
    pose_goal.orientation.y = my_data[frame].pose.orientation.y
    pose_goal.orientation.z = my_data[frame].pose.orientation.z
    pose_goal.orientation.w = my_data[frame].pose.orientation.w
    pose_goal.position.x = my_data[frame].pose.position.x  
    pose_goal.position.y = my_data[frame].pose.position.y 
    pose_goal.position.z = my_data[frame].pose.position.z - 0.01

    move_groupr.set_pose_target(pose_goal)
    plan_pick = move_groupr.plan()
    move_groupr.execute(plan_pick[1])

    joint_state = JointState()
    joint_state.name = move_groupr.get_active_joints()
    joint_state.position = move_groupr.get_current_joint_values()
    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = joint_state
    move_groupr.set_start_state(moveit_robot_state)
    ################################################################

    #################_THIRD STEP GO AGAIN OVER THE CUBE_#################
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = my_data[frame].pose.orientation.x
    pose_goal.orientation.y = my_data[frame].pose.orientation.y
    pose_goal.orientation.z = my_data[frame].pose.orientation.z
    pose_goal.orientation.w = my_data[frame].pose.orientation.w
    pose_goal.position.x = my_data[frame].pose.position.x  
    pose_goal.position.y = my_data[frame].pose.position.y 
    pose_goal.position.z = my_data[frame].pose.position.z + 0.2
        
    move_groupr.set_pose_target(pose_goal)
    plan_cubeup = move_groupr.plan()
    move_groupr.execute(plan_cubeup[1])

    joint_state = JointState()
    joint_state.name = move_groupr.get_active_joints()
    joint_state.position = move_groupr.get_current_joint_values()
    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = joint_state
    move_groupr.set_start_state(moveit_robot_state)
    ################################################################

    #################_FOURTH STEP PLACE IN THE MIDDLE_#################
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = my_data[23].pose.orientation.x
    pose_goal.orientation.y = my_data[23].pose.orientation.y
    pose_goal.orientation.z = my_data[23].pose.orientation.z
    pose_goal.orientation.w = my_data[23].pose.orientation.w
    pose_goal.position.x = my_data[23].pose.position.x  
    pose_goal.position.y = my_data[23].pose.position.y + 0.1
    pose_goal.position.z = my_data[23].pose.position.z + 0.1
        
    move_groupr.set_pose_target(pose_goal)
    plan_place = move_groupr.plan()
    move_groupr.execute(plan_place[1])

    #The baxter returns in its initial position 
    joint_state = JointState()
    joint_state.name = ['head_pan', 'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2', 'left_s0', 'left_s1', 'left_e0', 'left_e1', 
    'left_w0', 'left_w1', 'left_w2', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint']
    joint_state.position = [0.0, 0.5235987755982988, -1.2217304763960306, 0.0, 1.7278759594743864, 0.0, 0.7504915783575616, 0.0, -0.5235987755982988, 
    -1.2217304763960306, 0.0, 1.7278759594743864, 0.0, 0.7504915783575616, 0.0, 0.0, 0.0, 0.0, 0.0]
    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = joint_state
    move_groupr.set_start_state(moveit_robot_state)
    ################################################################

    #second field of baxter_msg to fill, and all the plans go inside it
    baxter_msg.trajectory.extend([plan_overcube[1], plan_pick[1], plan_cubeup[1], plan_place[1]])

    #publish the baxter_msg and see the simulation
    pub.publish(baxter_msg)
        

def main():
    global my_data, pub
    #moveit_commander's initialization
    moveit_commander.roscpp_initialize(sys.argv)

    #node's initialization with publisher to /baxter_moveit_trajectory for move the robot
    #and subscriber to /unity_tf for the positions of objects
    rospy.init_node('move_right_arm')
    pub = rospy.Publisher("/baxter_moveit_trajectory", BaxterTrajectory, queue_size=100)

    #For the collision avoidance, it needs to add a box to the scene
    #In this code the collision to avoid are the one with the table and the one with the human.
    #For the table we use a box and for the human a thin cube, a sort of "anti-COVID plexiglass"
    scene = moveit_commander.PlanningSceneInterface()
    time.sleep(2)

    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = "world"
    table_pose.pose.orientation.x = 0.0
    table_pose.pose.orientation.y = 0.0
    table_pose.pose.orientation.z = 0.0
    table_pose.pose.position.x = 0.7
    table_pose.pose.position.y = 0.0
    table_pose.pose.position.z = 0.4
    box_name = "table"
    scene.add_box(box_name, table_pose, size=(0.8, 2, 0.6))

    human_pose = geometry_msgs.msg.PoseStamped()
    human_pose.header.frame_id = "world"
    human_pose.pose.orientation.x = 0.0
    human_pose.pose.orientation.y = 0.0
    human_pose.pose.orientation.z = 0.0
    human_pose.pose.position.x = 1.1
    human_pose.pose.position.y = 0.0
    human_pose.pose.position.z = 1
    box_name2 = "table"
    scene.add_box(box_name2, human_pose, size=(0.05, 2, 1.5))


    rospy.Subscriber('/unity_tf', UnityTf, callback)
    time.sleep(1)

    #CUBES to pick and place
    E_cube = 11
    M_cube = 20
    move_right(E_cube)
    move_right(M_cube)
    
    
    
    

if __name__ == '__main__':
    main()
    