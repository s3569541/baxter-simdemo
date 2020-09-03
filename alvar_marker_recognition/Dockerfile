#FROM vxlab-red
FROM ros:melodic-ros-core

COPY 02proxy /etc/apt/apt.conf.d/02proxy

# Fix ROS keys
RUN apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Update apt because previous images clear this cache
# Commands are combined in single RUN statement with "apt/lists" folder removal to reduce image size
RUN apt update && \
    # Install some base dependencies
    apt install -y \
        telnet curl iproute2 host wget less sudo vim-tiny iputils-ping lsof psmisc iproute2 host git build-essential

# Ensure Ubuntu and ROS up to date
RUN apt-get update && apt -y dist-upgrade

USER root

RUN apt-get -qq update && \
    # Install some base dependencies
    apt-get -qq install -y \
        # Required for installing dependencies
        python-rosdep \
        # Preferred build tool
        python-catkin-tools
    # Download all dependencies

RUN [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init
RUN rosdep update

ENV CATKIN_WS=/root/ws_alvar
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Replacing shell with bash for later docker build commands
RUN mv /bin/sh /bin/sh-old && \
    ln -s /bin/bash /bin/sh

RUN apt update && apt install -y git
RUN git clone https://github.com/ros-perception/ar_track_alvar.git

WORKDIR $CATKIN_WS
COPY camera_controlx.py src/ar_track_alvar/ar_track_alvar/src/camera_controlx.py
ENV PYTHONIOENCODING UTF-8
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    ## Status rate is limited so that just enough info is shown to keep Docker from timing out, but not too much
    ## such that the Docker log gets too long (another form of timeout)
    rosdep update && \
    rosdep install -r -y --from-paths . && \
    catkin build --jobs 1 --limit-status-rate 0.1 --no-notify

RUN echo force rebuild
COPY alvar_baxter_vxlab.launch alvar_baxter_vxlab.launch
COPY vxlab_bundle.launch vxlab_bundle.launch
COPY start start
COPY startalvar startalvar
COPY bundles/block1bundle.xml bundles/block1bundle.xml
COPY bundles/block2bundle.xml bundles/block2bundle.xml
COPY bundles/block3bundle.xml bundles/block3bundle.xml
COPY bundles/block4bundle.xml bundles/block4bundle.xml
COPY bundles/block5bundle.xml bundles/block5bundle.xml

WORKDIR /root
COPY rosenv rosenv

RUN echo "source ~/rosenv" >> ~/.bashrc

COPY 02proxy /etc/apt/apt.conf.d/02proxy

# Launch file is here and referred to as "./"
WORKDIR $CATKIN_WS

#CMD ["/bin/bash","-c","/root/ws_alvar/start"]
#ENTRYPOINT [ "/bin/bash", "-c", "/root/ws_alvar/start" ]
ENTRYPOINT [ "/root/ws_alvar/start" ]
