"""
To move Graey in a square.
"""

#!/usr/bin/env python

import rospy
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool

from auv.motion import robot_control

class SquareMovement:
    def __init__(self):
        self.rate = rospy.Rate(10) # 10 Hz
        self.command_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size = 10)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.power = 2 # Adjust

    def arm(self):
        """Arm autonomous function"""
        try: 
            self.arm_service(True)
            rospy.loginfo("Vehicle armed.")
            return True
        
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to arm vehicle: %s" % e)
    
    def disarm(self):
        """Disarm autonomous function"""
        try:
            self.arm_service(False)
            rospy.loginfo("Vehicle disarmed.")
            return True
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to disarm vehicle: %s" % e)
    
    # def move_forward(self, power):
    #     """Move forward at a certain speed by changing the PWM value for a specific thruster (through channels)"""
    #     msg = OverrideRCIn()
    #     channels = [1500] * 18
    #     channels[4] = int((power * 80) + 1500)
    #     msg.channels = channels
    #     self.command_pub.publish(msg)

    # def yaw_right(self, power):
    #     """Yaw by changing PWM value for a specific thruster (through channels)"""
    #     msg = OverrideRCIn()
    #     channels = [1500] * 18
    #     channels[3] = int((power * 80) + 1500)
    #     msg.channels = channels
    #     self.command_pub.publish(msg)

    def stop(self):
        """Sets PWMs to neutral values in order to stop the sub"""
        msg = OverrideRCIn()
        msg.channels = [1500] * 18 # Neutral values
        self.command_pub.publish(msg)

    def move_in_square(self, power):
        """Move forward, then yaw. Repeat 4 times."""
        for i in range(4):
            rc = robot_control.RobotControl()
            rc.movement(forward = power)
            rospy.sleep(2.0) # Adjust
            self.stop()
            rospy.sleep(1.0)

            rc.movement(yaw = power)
            rospy.sleep(1.2) # Adjust
            self.stop()
            rospy.sleep(1.0)
    
    def run(self):
        """Arm autonomous function, move in the square, disarm autonomous function."""
        rospy.loginfo("Arming vehicle...")
        if self.arm():
            rospy.loginfo("Vehicle armed. Executing square movement.")
            self.move_in_square(self.power)
            rospy.loginfo("Square movement completed. Disarming.")
            self.disarm()
            rospy.loginfo("Vehicle disarmed.")
        else:
            rospy.logwarn("Vehicle arm failed. Exiting...")

if __name__ == '__main__':
    """For running the script in terminal"""
    rospy.init_node("test")
    try:
        square_movement = SquareMovement()
        square_movement.run()
    except rospy.ROSInterruptException:
        pass