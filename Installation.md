# ROS Installation

This code uses **ROS 1, Melodic** (supported May 2018 – May 2023 EOL). This version of ROS runs on **Ubuntu 18.04 Bionic Beaver**.

## 1. Install Ubuntu

Install Ubuntu on the relevant hardware platform.

### 1.1 For Jetson Xavier NX

Follow these steps: <https://developer.nvidia.com/embedded/learn/get-started-jetson-xavier-nx-devkit>. More NVIDIA resources can be found here: <https://developer.nvidia.com/embedded/downloads>. As of July 2021, the NVIDIA image for Xavier NX includes Ubuntu 18.04.

### 1.2 For AWS EC2 Instance

These steps assume you already have an AWS account, know how to launch an EC2 instance and edit the security group associated with an EC2 instance, and know how to connect to a remote computer using RDP.

1. Launch an EC2 instance with Ubuntu 18.04. For reference, my configuration was Ubuntu Server 18.04, t2.large, 64 GB storage. Make sure to save the private key file in a safe place.
2. Add inbound rules to the EC2 security group: port 22 (for SSH) and 3389 (for RDP). [More details](https://docs.aws.amazon.com/AWSEC2/latest/UserGuide/authorizing-access-to-an-instance.html).
3. Optional: Assign an elastic IP to the EC2 instance. This prevents the public IP of the instance from changing every time the instance is started. [More details](https://docs.aws.amazon.com/AWSEC2/latest/UserGuide/elastic-ip-addresses-eip.html#using-instance-addressing-eips-allocating).
4. SSH into the EC2 instance: `ssh -i /path/my-key-pair.pem my-instance-user-name@my-instance-public-dns-name` [More details](https://docs.aws.amazon.com/AWSEC2/latest/UserGuide/AccessingInstancesLinux.html). The remaining steps should be completed inside the EC2 instance.
5. Run `sudo apt-get update -y`
6. Install Xubuntu: `sudo apt install xubuntu-desktop -y` ([Reference](https://itsfoss.com/install-xfce-desktop-xubuntu/))
7. Install xrdp: `sudo apt-get install lxde -y`, then `sudo apt-get install xrdp -y` ([Reference](https://www.australtech.net/how-to-enable-gui-on-aws-ec2-ubuntu-server/))
8. Set a password for the default user "ubuntu": `sudo passwd ubuntu` ([Reference](https://www.australtech.net/how-to-enable-gui-on-aws-ec2-ubuntu-server/))
9. Setup Xubuntu with xrdp: ([Reference](https://www.tweaking4all.com/software/linux-software/use-xrdp-remote-access-ubuntu-14-04/))
   1. `echo xfce4-session > ~/.xsession`
   2. `sudo echo startxfce4 >> /etc/xrdp/startwm.sh`
   3. `sudo service xrdp restart`
10. Highly recommended: Fix xrdp speed issues as described here: <https://superuser.com/questions/1539900/slow-ubuntu-remote-desktop-using-xrdp>. Otherwise RDP will be extremely slow.
11. Exit SSH and connect to the EC2 instance from a RDP client.

### 1.3 For Mac

1. Follow this tutorial: <http://wiki.ros.org/docker/Tutorials/Docker>.
   1. Make sure to run `docker pull ros:melodic-robot` and not `docker pull ros:noetic-robot`

## 2. Install ROS and mavros

### 2.1 For Ubuntu (Jetson Xavier NX, AWS EC2 Ubuntu)

1. Follow the official ROS installation guide: <http://wiki.ros.org/melodic/Installation/Ubuntu>.
2. Follow the mavros documentation installation guide: <https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation>. Make sure to replace `kinetic` with `melodic`.

### 2.2 For Mac

#### 2.2.1 mavros.Dockerfile

This repository already has a Dockerfile that includes ROS and mavros: [mavros.Dockerfile](mavros.Dockerfile). To run a Docker container based off this Dockerfile:

1. `cd` into the repository
2. Run `docker build -t mavros -f mavros.Dockerfile .` (You only need to run this once. For future containers, you can just run step #3)
3. Run `docker run -it --rm -P mavros` to start an isolated container.
   1. Alternatively, run `docker run -it --rm -P --network kno_ros --env ROS_MASTER_URI=http://master:11311 mavros` to start a container connected to the Docker Compose network of containers. This command assumes Docker Compose is already running.

#### 2.2.2 docker-compose.yml

This [docker-compose.yml](docker-compose.yml) configuration launches a ROS master, a mavros node, and echos the `/mavros/state` topic to stdout.

1. `cd` into the repository
2. Run `docker-compose up`
3. See <http://wiki.ros.org/docker/Tutorials/Compose> for more details.

#### 2.2.3 catkin.Dockerfile

This repository also contains a second Dockerfile that does not include mavros but automatically sources a Catkin workspace: [catkin.Dockerfile](catkin.Dockerfile). It serves a small purpose of allowing VS Code to start a dev container with this repository inside the container, so that VS Code IntelliSense and the VS Code ROS extension can detect ROS packages like rospy and mavros.

## 3. Optional: Install ArduPilot SITL

### 3.1 For x86-Based Ubuntu (AWS EC2 Ubuntu)

1. Install ArduPilot: <https://ardupilot.org/dev/docs/building-setup-linux.html>.
   1. Run `./waf configure` to build all the SITL files. [More details](https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md).
2. Start the SITL: <https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html> and <https://www.ardusub.com/developers/sitl.html>.

### 3.2 For ARM-Based Ubuntu (Jetson Xavier NX)

Some dependencies of ArduPilot don't support ARM so you might encounter issues during installation (specifically pygame in my instance).

1. Follow steps for x86-based Ubuntu, but:
2. When installing ArduPilot, instead of `Tools/environment_install/install-prereqs-ubuntu.sh -y`, run `SKIP_AP_EXT_ENV ./Tools/environment_install/install-prereqs-ubuntu.sh -y`. This undocumented flag will skip pygame due to a Python version compatibility issue.

### 3.3 For x86-Based Mac (Pre-2020)

1. Install XQuartz, Vagrant, and VirtualBox.
2. Follow this tutorial: <https://gist.github.com/drnic/7684ce449ef5cfa965166ddd907dd36a>.

### 3.4 For ARM-Based Mac (Apple Silicon)

VirtualBox does not support Apple Silicon (and may never). You may try Parallels with [vagrant-parallels](https://github.com/Parallels/vagrant-parallels), but Parallels is not open-source and requires a paid subscription. In conclusion, it is currently not possible/practical to run a Linux-based graphical SITL on Apple Silicon.

## 4. Other Notes

- If `sim_vehicle.py` returns "command not found" then run `source ~/.profile`.
- Note that QGroundControl cannot be installed on the Jetson Xavier NX because there is no ARM support for QGroundControl or its dependencies (specifically Qt).
