#!/bin/bash

# Start roscore in a screen session
screen -dmS roscore bash -c 'roscore'

# Wait for roscore to start
sleep 3

# Source ROS setup.bash
screen -S roscore -X stuff 'source /opt/ros/noetic/setup.bash\n'

# Start mavros in another screen session
screen -dmS mavros bash -c 'source /opt/ros/noetic/setup.bash; roslaunch mavros px4.launch'

echo "ROS nodes are now running in separate screens."