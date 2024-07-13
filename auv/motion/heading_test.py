from . import robot_control




if __name__ == "__main__":
    rc = robot_control.RobotControl()
    forward = float(input("Get forward:"))
    rc.movement(forward=forward)
    forward = float(input("Get forward:"))
    rc.movement(forward=forward)
    yaw = float(input("Get yaw:"))
    rc.movement(yaw=yaw)
    yaw = float(input("Get yaw:"))
    rc.movement(yaw=yaw)
    lateral = float(input("Get lateral:"))
    rc.movement(lateral=lateral)
    lateral = float(input("Get lateral:"))
    rc.movement(lateral=lateral)
    forward = float(input("Get forward:"))
    rc.movement(forward=forward)
    forward = float(input("Get forward:"))
    rc.movement(forward=forward)
    