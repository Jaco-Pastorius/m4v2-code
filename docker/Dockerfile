FROM ros:foxy

# Basic packages
RUN apt-get update && apt-get install --no-install-recommends -y \
    vim

# install bootstrap tools
RUN apt-get install --no-install-recommends -y \
    python3-pip

RUN apt-get install --no-install-recommends -y \
    ros-foxy-vrpn-mocap

# install pyserial 
RUN python3 -m pip install pyserial 

# Set home directory - Mount this when running
WORKDIR /home/m4version2
