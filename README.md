ROS LEAP MOTION
=============

Leap Motion ROS integration


REQUIREMENTS
============

ROS Groovy installed including rospy and geometry_msg and the LEAP MOTION SDK for Linux


INSTALLATION
==============

1. If you don't already have a catkin workspace, please follow these instructions before starting with 2.: http://www.ros.org/wiki/catkin/Tutorials/create_a_workspace

2. cd ~/catkin_ws/src

3. git clone https://github.com/warp1337/rosleapmotion.git

4. cd ~/catkin_ws && catkin_make

5. Start a roscore (another shell) and leapd (another shell)

6. You need to append the location of your LeapSDK (especially /lib and /lib/x64 or x86) to your PYTHONPATH,e.g., export PYTHONPATH=$PYTHONPATH:/path/to/SDK/lib:/path/to/SDK/lib/x64
Remember that you will need to have your path set at least in the "sender" shell. If you don't want to set it every time, you can also alter the leapinterface.py file (have a look at it).

6. source ~/catkin_ws/devel/setup.bash && rosrun leap_motion sender.py (another shell)

7. source ~/catkin_ws/devel/setup.bash && rosrun leap_motion subscriber.py (another shell) 

8. You are done, you should see the LEAP MOTION coordinates in your shell prompt


USE LEAP AS STEREO CAMERA
============================
if you want to use leap_motion as stereo_camera, You can use by compiling it.

1. Make sure that You are using LeapSDK ver 2.*

2. You need to append the location of your LeapSDK (especially /lib and /lib/x64 or x86) to your LEAP_SDK_PATH,e.g., add "export LEAP_SDK=/home/Your_name/LeapDebeloperKit_2.*/LeapSDK " in ~/.bashrc

3. command LeapControlPanel in your shell and enable the feature in the Leap Motion control panel for any application to get the raw camera images.

4. in your catkin_ws, catkin_make --only-pkg-with-depth leap_motion

5. roslaunch leap_motion leap_camera.launch

6. roslaunch leap_motion leap_stereo.launch

