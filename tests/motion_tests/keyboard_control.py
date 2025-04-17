#!/usr/bin/env python3

import rospy
import sys
import select
import termios
import tty
from auv.motion import robot_control
from auv.utils import arm, disarm

# Key mappings
move_bindings = {
    'w': {'forward':    0.5},
    's': {'forward':   -0.5},
    'a': {'lateral':   -0.5},
    'd': {'lateral':    0.5},
    'q': {'yaw':        0.5},
    'e': {'yaw':       -0.5},
    'z': {'vertical':  -0.5},
    'x': {'vertical':   0.5},
}

def get_key(timeout=0.1):
    """Get a single keypress from the terminal with timeout."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("keyboard_control_node")
    rc = robot_control.RobotControl(enable_dvl=False)

    rospy.loginfo("Arming...")
    arm.arm()
    rospy.sleep(2.0)

    rospy.loginfo("Use WASD to move, Q/E to yaw, Z/X for vertical, Ctrl+C to quit.")
    
    try:
        while not rospy.is_shutdown():
            key = get_key()
            motion_args = {}
            print(f"You pressed: {key}")
            if key in move_bindings:
                motion_args = move_bindings[key]
                rc.movement(**motion_args)
            elif key == '\x03':  # Ctrl+C
                break
            else:
                rc.movement(0, 0, 0, 0)  # Stop

    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Disarming...")
        disarm.disarm()
        rc.movement(0, 0, 0, 0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rospy.loginfo("Shutdown complete.")
