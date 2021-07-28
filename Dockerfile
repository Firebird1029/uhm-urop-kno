# INSTRUCTIONS:
# Run `docker pull ros:noetic-robot` in the command line.
# Run `docker build -t mavros .` (in the same folder as this Dockerfile)
# Run `docker run -it --rm -P ros`
#
# See http://wiki.ros.org/docker/Tutorials/Docker for more details.
# This Dockerfile is based on https://hub.docker.com/r/droneemployee/developer/dockerfile.

FROM ros:melodic-robot

# install mavros
RUN sudo apt-get update && \
	sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras -y
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
	chmod +x install_geographiclib_datasets.sh && \
	./install_geographiclib_datasets.sh

# setup container
EXPOSE 14550/udp

# source ROS entrypoint
SHELL ["/bin/bash", "-c"]
RUN source ros_entrypoint.sh
