#!/usr/bin/env python 

# Further and detailed documentation for used functions and structures is at this links:
#http://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html#a3b8394f6bc398e43e22787de15041afd
#https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py

import sys
import moveit_commander 
import geometry_msgs
from moveit_msgs.msg import RobotState                                                        
from sensor_msgs.msg import JointState

#####################################################################################################
# MOVEIT_COMMANDER.ROSCPP_INITIALIZE(SYS.ARGV):                                                     #
# Needed to initialize moveit_commander                                                             #
#####################################################################################################

moveit_commander.roscpp_initialize(sys.argv)



#####################################################################################################
# MOVEIT_COMMANDER.MOVEGROUPCOMMANDER():                                                            #
# Instantiate a `MoveGroupCommander`_ object. This object is an interface                           #
# to a planning group (group of joints). In our case the groups are left_arm and right_arm          #
#####################################################################################################

moveit_commander.MoveGroupCommander('right_arm')
moveit_commander.MoveGroupCommander('left_arm')

#####################################################################################################
# MOVE_GROUP.SET_START_STATE():                                                                     #
# specify a start state for the group                                                               #
# Examples                                                                                          #
# --------                                                                                          #
# >>> from moveit_msgs.msg import RobotState                                                        #
# >>> from sensor_msgs.msg import JointState                                                        #
# >>> joint_state = JointState()                                                                    #
# >>> joint_state.header = Header()                                                                 #
# >>> joint_state.header.stamp = rospy.Time.now()                                                   #
# >>> joint_state.name = ['joint_a', 'joint_b']                                                     #
# >>> joint_state.position = [0.17, 0.34]                                                           #
# >>> moveit_robot_state = RobotState()                                                             #
# >>> moveit_robot_state.joint_state = joint_state                                                  #
# >>> move_group.set_start_state(moveit_robot_state)                                                #
#                                                                                                   #
# MOVE_GROUP.GET_ACTIVE_JOINTS():                                                                   #
# Get the active joints of the group                                                                #
#                                                                                                   #
# MOVE_GROUP.GET_CURRENT_JOINT_VALUES():                                                            #
# Get the current configuration of the group as a list                                              #
#####################################################################################################

move_group = moveit_commander.MoveGroupCommander('right_arm')
joint_state = JointState()
joint_state.name = move_group.get_active_joints()
joint_state.position = move_group.get_current_joint_values()
moveit_robot_state = RobotState()
moveit_robot_state.joint_state = joint_state
move_group.set_start_state(moveit_robot_state)

#####################################################################################################
# MOVE_GROUP.SET_POSE_TARGET():                                                                     #
# Set the pose of the end-effector, if one is available. The expected input is a                    #
# Pose message, a PoseStamped message or a list of 6 floats.                                        #
# In our code we have a pose, so 7 parameters to fill                                               #
#####################################################################################################

pose_goal = geometry_msgs.msg.Pose()
move_group.set_pose_target(pose_goal)

#####################################################################################################
# MOVE_GROUP.PLAN():                                                                                #
# Return a motion plan (a RobotTrajectory in the first field of the list) to the set goal state     #
#                                                                                                   #
# MOVE_GROUP.EXECUTE(PLAN[1]):                                                                      #
# Execute a previous planned path                                                                   #
#####################################################################################################

plan = move_group.plan()
move_group.execute(plan[1])

#####################################################################################################
# MOVEIT_COMMANDER.PLANNINGSCENEINTERFACE():                                                        #
# Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface                 #
# for getting, setting, and updating the robot's internal understanding of the surrounding world    #
#                                                                                                   #
# SCENE.ADD_BOX(BOX_NAME, BOX_POSE, BOX_SIZE):                                                      #
# Add in the scene a box, defined with name, position and size, for collision                       #
#####################################################################################################

scene = moveit_commander.PlanningSceneInterface()
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