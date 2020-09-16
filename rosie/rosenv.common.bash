#!/bin/bash

# install-specific config
source ~/rosie/rosenv.local.bash

HOSTNAME=`hostname`
export ROS_IP=`host ${HOSTNAME} | sed 's/.* //g'`

#MASTERNAME=`echo $ROS_MASTER_URI | sed 's%http://%%' | sed 's%:11311%%'`
#MASTER_IP=`host ${MASTERNAME} | sed 's/.* //g'`
#echo HOSTNAME $HOSTNAME 
#echo MASTERNAME $MASTERNAME
#if [ "$HOSTNAME" = "$MASTERNAME" ]; then
#    echo "Is MASTER"
#    #export ROS_MASTER_URI=http://localhost:11311
#else
#    echo "Remote Master"
#fi
#echo ROS_MASTER_URI $ROS_MASTER_URI

export MB_LASER_BIRDCAGE_R2000=1
export MB_LASER_BIRDCAGE_R2000_FREQ=50
export MB_LASER_BIRDCAGE_R2000_SAMPLES=3600

export LIBGL_ALWAYS_INDIRECT=0

export PATH=$PATH:~/rosie
export GAZEBO_MODEL_PATH=~/rosie
