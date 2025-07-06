"""
This is the servo node that controls all servos connected to our pololo mini maestro
Service names:
    - dropper : /auv/device/dropper
    - gripper : /auv/device/gripper
    - torpedo:  /auv/device/torpedo
"""
import rospy
import time
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from auv.device.MiniMaestro.mini_maestro_api import MiniMaestro
class MaestroServer:
    def __init__(self):
        rospy.init_node('maestroServer')
        self.maestro = MiniMaestro()
        self.dropperService = rospy.Service('/auv/device/dropper', Trigger, self.dropperCallback)
        self.gripperService = rospy.Service('/auv/device/gripper', Trigger, self.gripperCallback)
        self.torpedoService = rospy.Service('/auv/device/torpedo', Trigger, self.torpedoCallback)
        rospy.loginfo("Ready to add two ints.")
        rospy.spin()

        # TODO setting all servos to default
        
        
    def dropperCallback(self):
        rospy.loginfo("dropping a marker")

        # Logic for droping one marker
        self.maestro.set_pwm(1,1800)
        time.sleep(0.5) # TODO find time for dropping only one marker
        self.maestro.set_pwm(1,1450)

        return TriggerResponse(
            success=True,
            message="Marker dropped !"
        )


    def gripperCallback(self):
        rospy.loginfo("Gripper has been triggered")

        return TriggerResponse(
            success=True,
            message="Triggered successfully!"
        )


    def torpedoCallback(self):
        rospy.loginfo("launching torpedo")

        return TriggerResponse(
            success=True,
            message="Torpedo launched !"
        )


