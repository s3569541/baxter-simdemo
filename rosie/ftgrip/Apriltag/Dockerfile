FROM ros:kinetic-ros-base-xenial

ENV CATKIN_WS=/root/ws
RUN mkdir -p $CATKIN_WS/src

WORKDIR $CATKIN_WS/src

RUN git clone https://github.com/AprilRobotics/apriltag.git
RUN git clone https://github.com/AprilRobotics/apriltag_ros.git

WORKDIR $CATKIN_WS
RUN apt-get update && apt-get install -q -y iproute2
RUN rosdep install --from-paths src --ignore-src -r -y
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make_isolated"

RUN apt-get update && apt-get install -q -y nano vim
RUN apt-get update && apt-get dist-upgrade -q -y

COPY right_hand_detection.launch $CATKIN_WS/src/apriltag_ros/apriltag_ros/launch/right_hand_detection.launch
COPY right_hand_settings.yaml $CATKIN_WS/src/apriltag_ros/apriltag_ros/config/right_hand_settings.yaml
COPY right_hand_tags.yaml $CATKIN_WS/src/apriltag_ros/apriltag_ros/config/right_hand_tags.yaml
COPY start-april-ftgrip $CATKIN_WS

