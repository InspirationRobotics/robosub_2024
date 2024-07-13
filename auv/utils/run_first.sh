#!/bin/bash

# Start roscore in a screen session
screen -dmS roscore bash -c 'roscore'

# Wait for roscore to start
sleep 3

# Source ROS setup.bash
screen -S roscore -X stuff 'source /opt/ros/noetic/setup.bash\n'

# Start MAVROS in another screen session
screen -dmS mavros bash -c 'source /opt/ros/noetic/setup.bash; roslaunch mavros px4.launch'

# Wait for MAVROS to start
sleep 3

# Update the RoboSub repository to the latest version
cd /home/inspiration/auv
git pull

# Run camsVersatile
screen -dmS cams bash -c 'cd /home/inspiration/auv; python3 -m auv.device.camsVersatile'

# Run pix_standalone
screen -dmS pix bash -c 'cd /home/inspiration/auv; python3 -m auv.device.pix_standalone'

# Wait for pix_standalone and camsVersatile to be up and running
sleep 3

echo "ROS nodes and dependencies for autonomous missions are now running in separate screens."
