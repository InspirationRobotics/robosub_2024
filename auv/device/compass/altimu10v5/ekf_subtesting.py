import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from auv.device.dvl import DVL  
from positionEKF import PoseEKF
import time

def main():
    # Initialize ROS node
    rospy.init_node('pose_ekf_node', anonymous=True)

    # Initialize DVL object (replace with actual DVL initialization)
    dvl = DVL(sub_type="onyx")  # Example: Using Onyx DVL

    # Initialize the PoseEKF class with real sensor data
    pose_ekf = PoseEKF(dvl, use_simulated_data=False)

    # Main loop to continuously update the EKF
    try:
        while not rospy.is_shutdown():
            # Update the EKF with the latest sensor data
            pose_ekf.update_filter()

            # Get the estimated velocity and position uncertainty
            estimated_velocity = pose_ekf.get_estimated_velocity()
            position_uncertainty = pose_ekf.get_position_uncertainty()

            # Print the results (for demonstration purposes)
            print(f"Estimated Velocity: {estimated_velocity}")
            print(f"Position Uncertainty: {position_uncertainty}")

            # Sleep for a short duration to simulate real-time operation
            time.sleep(0.01)  # 100 Hz update rate

    except rospy.ROSInterruptException:
        print("EKF loop terminated by ROS shutdown.")

if __name__ == "__main__":
    main()