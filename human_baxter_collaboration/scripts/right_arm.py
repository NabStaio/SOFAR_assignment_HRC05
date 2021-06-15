#!/usr/bin/env python 
#Guys WE GOT THE PICK AND PLACE!!!
#The code for now is only a pick and place for cube C, it miss the pick and place for the others cubes
#and the E that need the help of right arm


#libraries 
from __future__ import print_function

import rospy
import sys
import copy
import time
import moveit_commander 
import moveit_msgs.msg
import geometry_msgs.msg 
import trajectory_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState, RobotTrajectory
from geometry_msgs.msg import Quaternion, Pose, PoseStamped
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from human_baxter_collaboration.msg import BaxterTrajectory, UnityTf

global my_data

#callback for subscriber that we need for position of cubes and bluebox
def callback(data):
    global my_data
    my_data = data.frames
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.frames)


def move_right(cube):
    global my_data, pub

    frame = cube

    move_group = moveit_commander.MoveGroupCommander('right_arm')
    #first field of baxter_msg, the arm 
    baxter_msg = BaxterTrajectory()
    baxter_msg.arm = 'right'


    #setting of the initial state of the robot
    joint_state = JointState()
    joint_state.name = ['head_pan', 'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2', 'left_s0', 'left_s1', 'left_e0', 'left_e1', 
    'left_w0', 'left_w1', 'left_w2', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint']
    joint_state.position = [0.0, 0.5235987755982988, -1.2217304763960306, 0.0, 1.7278759594743864, 0.0, 0.7504915783575616, 0.0, -0.5235987755982988, 
    -1.2217304763960306, 0.0, 1.7278759594743864, 0.0, 0.7504915783575616, 0.0, 0.0, 0.0, 0.0, 0.0]
    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = joint_state
    move_group.set_start_state(moveit_robot_state)
    
        
    #Now the planning step 
    #1st go over the cube
    #2nd pick the cube
    #3rd go up
    #4th place the cube in the bluebox
    #for doing each steps we need first the pose_goal of the robot, then do a plan&execute and
    #for updating the position we neeed the last position
    
    #################_FIRST STEP GO OVER THE CUBE_#################
    print("cube position: %s", my_data[frame])
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = my_data[frame].pose.orientation.x
    pose_goal.orientation.y = my_data[frame].pose.orientation.y
    pose_goal.orientation.z = my_data[frame].pose.orientation.z
    pose_goal.orientation.w = my_data[frame].pose.orientation.w
    pose_goal.position.x = my_data[frame].pose.position.x  
    pose_goal.position.y = my_data[frame].pose.position.y 
    pose_goal.position.z = my_data[frame].pose.position.z + 0.2

    move_group.set_pose_target(pose_goal)
    plan_overcube = move_group.plan()
    move_group.execute(plan_overcube[1])

    joint_state = JointState()
    joint_state.name = move_group.get_active_joints()
    joint_state.position = move_group.get_current_joint_values()
    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = joint_state
    move_group.set_start_state(moveit_robot_state)
    ################################################################

    #################_SECOND STEP PICK THE CUBE_#################
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = my_data[frame].pose.orientation.x
    pose_goal.orientation.y = my_data[frame].pose.orientation.y
    pose_goal.orientation.z = my_data[frame].pose.orientation.z
    pose_goal.orientation.w = my_data[frame].pose.orientation.w
    pose_goal.position.x = my_data[frame].pose.position.x  
    pose_goal.position.y = my_data[frame].pose.position.y 
    pose_goal.position.z = my_data[frame].pose.position.z - 0.02

    move_group.set_pose_target(pose_goal)
    plan_pick = move_group.plan()
    move_group.execute(plan_pick[1])

    joint_state = JointState()
    joint_state.name = move_group.get_active_joints()
    joint_state.position = move_group.get_current_joint_values()
    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = joint_state
    move_group.set_start_state(moveit_robot_state)
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
        
    move_group.set_pose_target(pose_goal)
    plan_cubeup = move_group.plan()
    move_group.execute(plan_cubeup[1])

    joint_state = JointState()
    joint_state.name = move_group.get_active_joints()
    joint_state.position = move_group.get_current_joint_values()
    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = joint_state
    move_group.set_start_state(moveit_robot_state)
    ################################################################

    #################_FOURTH STEP PLACE IN THE BLUEBOX_#################
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = my_data[23].pose.orientation.x
    pose_goal.orientation.y = my_data[23].pose.orientation.y
    pose_goal.orientation.z = my_data[23].pose.orientation.z
    pose_goal.orientation.w = my_data[23].pose.orientation.w
    pose_goal.position.x = my_data[23].pose.position.x  
    pose_goal.position.y = my_data[23].pose.position.y 
    pose_goal.position.z = my_data[23].pose.position.z + 0.1
        
    move_group.set_pose_target(pose_goal)
    plan_place = move_group.plan()
    move_group.execute(plan_place[1])

    joint_state = JointState()
    joint_state.name = move_group.get_active_joints()
    joint_state.position = move_group.get_current_joint_values()
    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = joint_state
    move_group.set_start_state(moveit_robot_state)
    ################################################################

    #second field of baxter_msg to fill, and all the plans go inside it
    baxter_msg.trajectory.extend([plan_overcube[1], plan_pick[1], plan_cubeup[1], plan_place[1]])

    #publish the message and see the simulation
    #time.sleep(10)
    
    pub.publish(baxter_msg)
        




#our main function
def main():
    global my_data, pub
    #moveit_commander's initialization
    moveit_commander.roscpp_initialize(sys.argv)
    #node's initialization with publisher to /baxter_moveit_trajectory for move the robot
    #and subscriber to /unity_tf for the positions of objects
    rospy.init_node('move_right_arm')
    pub = rospy.Publisher("/baxter_moveit_trajectory", BaxterTrajectory, queue_size=100)
    scene = moveit_commander.PlanningSceneInterface()
    time.sleep(2)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "world"
    box_pose.pose.orientation.x = 0.0
    box_pose.pose.orientation.y = 0.0
    box_pose.pose.orientation.z = 0.0
    box_pose.pose.position.x = 0.0
    box_pose.pose.position.y = 0.0
    box_pose.pose.position.z = 0.0
    box_name = "table"
    scene.add_box(box_name, box_pose, size=(1, 1, 1))
    rospy.Subscriber('/unity_tf', UnityTf, callback)
    time.sleep(1)
    
    
    #variables for logging robot state and the move_group that needs for moving right or left arm
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    E_cube = 11
    M_cube = 20
    move_right(E_cube)
    move_right(M_cube)
    
    
    
    

if __name__ == '__main__':
    main()
    