# INSTRUCTIONS:
# Run `docker pull ros:noetic-robot` in the command line.
# Run `docker build -t catkin -f catkin.Dockerfile .` (in the same folder as this Dockerfile)
# Run `docker run -it --rm -P -v /path/to/local/KNO/catkin_ws:/root/catkin_ws catkin` to launch an isolated container.
# Run `docker run -it --rm -P -v /path/to/local/KNO/catkin_ws:/root/catkin_ws --network kno_ros --env ROS_MASTER_URI=http://master:11311 catkin` to launch a container connected to the Docker Compose network of containers.
#	 Or run `docker exec -it kno_catkin_1 bash` to connect to the existing catkin container running in Docker Compose.
#
# See http://wiki.ros.org/docker/Tutorials/Docker for more details.
# This Dockerfile is based on https://hub.docker.com/r/droneemployee/developer/dockerfile.

FROM ros:melodic-robot

# source ROS entrypoint
SHELL ["/bin/bash", "-c"]
RUN source ros_entrypoint.sh

# helpful Bash aliases
RUN echo 'alias rosinit="cd ~/catkin_ws && source devel/setup.bash"' >> ~/.bashrc
RUN echo 'alias rosbuild="cd ~/catkin_ws && catkin_make && source devel/setup.bash"' >> ~/.bashrc

# source Catkin workspace
CMD /bin/bash -c "source ros_entrypoint.sh && source /root/catkin_ws/devel/setup.bash"; /bin/bash
