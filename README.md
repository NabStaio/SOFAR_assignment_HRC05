# SOFAR_assignment_HRC05
To start the simulation of the collaborative task follow these steps:
To get started you need a PC with Windows 10 installed and an Ubuntu 20.04 set-up using a VM, Docker
or using the dual PC set-up.
On UBUNTU:
1. Install ROS Noetic on Ubuntu 20.04- http://wiki.ros.org/noetic/Installation/Ubuntu
2. Create a Catkin workspace for this project
3. Visit this https://github.com/TheEngineRoom-UniGe/SofAR-Human-Robot-Collaboration and
download the ROS packages needed for the project
4. Extract the packages to your catkin_ws
5. Open the 'human_baxter_collaboration' package, then navigate to the config folder and open the
params.yaml file. Under ROS_IP, insert the IP address of your machine.
6. Compile packages with catkin_make
7. Don't forget you need MoveIt framework for this project, thus visit https://moveit.ros.org/install/
and install the binaries
8. Once all dependencies are resolved, do: 'roslaunch human_baxter_collaboration
human_baxter_collaboration.launch' to start MoveIt and communication with Unity (ensure the
server communication is up and running, you should see the following line in the terminal
'Starting server on YOUR IP:10000')
On WINDOWS:
1. Visit https://unity3d.com/get-unity/download and download Unity Hub and inside that install
Unity 2020.2.2 (the version for which the projects have been developed) or later.
2. In order to install the framework for human simulation, visit
https://github.com/Daimler/mosim_core/wiki/InstallPrecompiled and follow the steps to install
the precompiled framework.
3. Visit https://github.com/TheEngineRoom-UniGe/SofAR-Human-Robot-Collaboration to download
the Unity project folder, extract it, then open Unity Hub and ADD the project to your projects list
using the associated button.
4. Open the project
5. In the bar on top of the screen, open the Ros-TCP-Connection tab and replace the ROS_IP with
the IP of the machine running ROS.
6. If the previous steps have been successful, you should be able to enter Editor mode (via the Play
button) and play the simulation.
7. If the communication is running correctly, on UBUNTU you can echo the topics that are being
exchanged between ROS and Unity.
(Macciò 2021)
To run our project follow these steps, after downloading it on:
https://github.com/NabStaio/SOFAR_assignment_HRC05
1. On the Ubuntu side, open a terminal and go to your catkin workspace (eg: catkin_ws) and the
'roslaunch human_baxter_collaboration human_baxter_collaboration.launch' to start MoveIt and
communicate with Unity. You will see a dialogue at the end of the terminal “ You can start
planning now!”
2. Then, you have to start the simulation on Unity by pressing the play button at the top and then the
“start simulation” button at the bottom.
3. Then, open the second terminal in your catkin_ws run “human_baxter_collaboration
moveC_arm.py”, the python script for the left arm of Baxter to pick and place the first cube, C,
and on the Unity side press on “pick and place button”, to start the human simulation.
4. When the left arm is done you can start the second program, in your catkin_ws run
“human_baxter_collaboration move_right_arm.py”, the python script for the right arm of Baxter.
5. Lastly, when the right arm has stopped you can, inside the catkin_ws, run the code for the left arm
“rosrun human_baxter_collaboration move_left_arm.py”
Tips
1. If you experience a strange window when opening the Unity Project for the first time, simply do
Quit, then reopen the project and it should not bother you further.
2. Should you experience issues in the communication between ROS and Unity, especially when
publishing from ROS to Unity, remember to disable the firewall of your PC, as it could interfere
with the sending of messages over TCP.
3. When running your project, be sure to launch the ROS files BEFORE playing the Unity
simulation, in order to have the server endpoint node up and running before initializing
communication.
4. In the fourth terminal inside our workspace you can launch “rviz” to see the simulation also on
ROS. Once the Rviz opens do the following steps:
a. If you are doing this for the first time, you should see an empty world in RViz and will
have to add the Motion Planning Plugin. You should see an empty world in RViz: choose
as frame: <world>
b. From the folder choose ‘Motion Planning’ as the display types and Press ‘OK’
c. Wait for some time to see the robot (later, if you wish to see the arm motion the goto
“planned path” and then click on “loop animation” and “show trail”.
