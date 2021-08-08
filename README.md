# KNO RAUV

## Kilo Nalu Observatory: Resident Autonomous Underwater Vehicle

[RAN Lab](http://www2.hawaii.edu/~zsong/) at University of Hawaii Manoa

---

## Abstract

This study explores various methods of implementing machine learning-based docking control algorithms for a custom AUV, or autonomous underwater vehicle. Unlike remotely operated underwater vehicles (ROVs), AUVs must autonomously navigate to a seafloor docking station to fetch power and network. We analyze the performance of different autonomy strategies on a modified Blue Robotics BlueROV2 controlled by ArduSub, an open-source firmware package for ROVs and AUVs. Some of the software used to perform this analysis include Robot Operating System (ROS), a widely-used set of open-source middleware libraries for robotics applications; Gazebo, a 3D robotics simulator; and a ROS package mavros, used to communicate with the MAVlink-based ArduSub firmware. To aid in the testing of docking control procedures, we utilize simulation software to display and evaluate the results of various machine learning methods. Taking advantage of simulation-based environments enables us to test proof of concept under a controlled setting and increase the efficiency of assessing multiple strategies. The results of this study will contribute to the foundation of a shared, open-access marine collaborative robot testbed at Kilo Nalu Observatory. The long-term vision includes establishing a first-of-its-kind open-water test site for resident AUVs to accelerate research of related disciplines, including oceanography, marine biology, and ocean and coastal engineering at the University of Hawaii.

## Setup

1. Install ROS on the desired hardware platform. See [Installation](Installation.md) steps for details on installation for a Jetson Xavier NX, a Mac/Windows (using Docker), or a AWS EC2 instance (cloud environment instance). **The code in this repository uses *ROS 1, Melodic* (supported May 2018 – May 2023 EOL). This version of ROS runs on *Ubuntu 18.04 Bionic Beaver*.**
2. Install mavros by following the mavros documentation installation guide: <https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation>. Make sure to replace `kinetic` with `melodic`.
3. Create a ROS catkin workspace if one doesn't exist already: <http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment>.
4. Clone this repo: `$ git clone https://github.com/song-ranlab/kno-rauv.git`
5. Create a symbolic $ link that links the KNO package to within the catkin workspace:
   1. `$ cd ~/catkin_ws/src`
   2. `$ ln -s /path/to/kno-rauv/catkin_ws/src/kno kno`
6. Confirm that the KNO package compiles correctly: `$ cd ~/catkin_ws && catkin_make`

## Test mavros

These steps are optional but are recommended to help confirm that mavros can successfully connect to the ROV (either a live ROV or the ArduSub SITL), send thruster control, and read data from sensors.

Note: If you are using the [docker-compose.yml](docker-compose.yml) configuration, you can skip steps #1-2.

1. Start ROS master: `$ roscore`
2. Start mavros: `$ roslaunch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@`
   1. Don’t forget the @ symbol, which specifies the end of a UDP URL.
   2. If it is stuck on “MAVROS started” then it did not connect successfully to the ROV and is still waiting for a connection.
   3. Do not use `/dev/ttyACM0` unless the Pixhawk is directly connected to the computer (not through the Raspberry Pi) via serial connection. In the context of ArduSub (which uses an Ethernet tether) and this project, that is unlikely the case.
3. Optional: Double check that the ROV is connected: `$ rostopic echo /mavros/state`. It should say “connected: True” in the output.
4. Make sure the `SYSID_MYGCS` parameter is set to 1 so that ROS can override manual control of the ROV: `$ rosrun mavros mavparam set SYSID_MYGCS 1`
   1. Optional: Double check with `$ rosrun mavros mavparam get SYSID_MYGCS`
   2. Optional: Set back to 255 after using mavros to allow QGroundControl and pymavlink to control the ROV
5. Set robot to Manual mode if not on Manual mode already: `$ rosrun mavros mavsys mode -c MANUAL`
6. Arm ROV: `$ rosrun mavros mavsafety arm`
7. Add some forward: `$ rostopic pub -r 1 /mavros/rc/override mavros_msgs/OverrideRCIn "channels: [1500, 1500, 1500, 1500, 1600, 1500, 1500, 1500]"`
   1. `-r 1` means publish at 1 Hz (1 signal every 1 second), ROS default is 10 Hz (10 signals every 1 second)
   2. The 8 numbers correspond to pitch, roll, throttle, yaw, ... as described here: <http://www.ardusub.com/developers/rc-input-and-output.html>
   3. Possible PWM values are 1100-1900, midpoint is 1500, release is 0, no change is 65535 [Reference 1](http://www.ardusub.com/developers/pymavlink.html#send-rc-joystick), [Reference 2](http://docs.ros.org/en/api/mavros_msgs/html/msg/OverrideRCIn.html)
   4. Note that the live ROV disarms itself if it does not receive RC input after 3.0 seconds after the first RC input command for safety reasons ([source](https://discuss.bluerobotics.com/t/bluerov2-constantly-disarming-itself-in-mavproxy/1232))
      1. Can be adjusted/turned off [here](http://www.ardusub.com/developers/full-parameter-list.html#fspilottimeout-timeout-for-activation-of-pilot-input-failsafe), e.g. `$ rosrun mavros mavparam set FS_PILOT_TIMEOUT 2.0`
8. Test sensor input data: `$ rostopic echo /mavros/global_position/compass_hdg`
   1. If it does not output anything, try: `$ rosservice call /mavros/set_stream_rate 0 10 1` ([source](https://answers.ros.org/question/193411/mavros-topics-not-publishing/))

## Running the RAUV node

Note: If you are using the [docker-compose.yml](docker-compose.yml) configuration, you can skip steps #1-2.

1. Start ROS master: `$ roscore`
2. Start mavros: `$ roslaunch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@`
3. Start the KNO RAUV node: `$ rosrun kno rauv.py`

## Examples

Note: If you are using the [docker-compose.yml](docker-compose.yml) configuration, you can skip steps #1-2.

1. Start ROS master: `$ roscore`
2. Start mavros: `$ roslaunch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@`
3. Start the KNO RAUV node: `$ rosrun kno rauv.py`
4. Run one of the examples:
   1. [1_arm.py](catkin_ws/src/kno/scripts/examples/1_arm.py): `$ rosrun kno 1_arm.py`
      1. This arms the ROV.
   2. [2_forward.py](catkin_ws/src/kno/scripts/examples/2_forward.py): `$ rosrun kno 2_forward.py`
      1. This arms the ROV and sends manual forward thruster control.
   3. [3_circle.py](catkin_ws/src/kno/scripts/examples/3_circle.py): `$ rosrun kno 3_circle.py`
      1. This will send manual RC joystick control to make the ROV move in a circular pattern (although more octagonal than circular).
   4. [4_simple_waypoints.py](catkin_ws/src/kno/scripts/examples/4_simple_waypoints.py): `$ rosrun kno 4_simple_waypoints.py`
      1. This tells the ROV to move forward by giving it a set of three waypoints (takeoff, GPS waypoint, land). It works in SITL, which contains functional GPS, but has not been tested on a live ROV yet.
   5. [5_figure_eight_waypoints.py](catkin_ws/src/kno/scripts/examples/5_figure_eight_waypoints.py): `$ rosrun kno 5_figure_eight_waypoints.py`
      1. This is still a work in progress. It sends a list of 21 waypoints to create a figure eight pattern, but due to ArduSub's lack of full waypoint mission autonomy functionality, it can only go to one waypoint at a time. As a result, it is the code's responsibility to 1. detect when the ROV has arrived at a waypoint and 2. tell the ROV to continue to the next waypoint in the list.

## SITL

Instructions on setting up the ArduSub SITL are described in the [Installation](Installation.md#3-optional-install-ardupilot-sitl) instructions.
