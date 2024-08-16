import math

from auv.motion import robot_control


class Waypoints:
    """This class exists to make localization capabilities in imitation of Si Se Puede. The whole
    course is converted to coordinate waypoints relative to the initial position, and we will 
    use the next() method to navigate through them. Be warned that this implementation 
    assumes little to no translational motion occurs during completion of missions"""
    def __init__(self, polar=True):
        """Initialize all the waypoints, the initial heading and the robot control object. Sets
        self.polar to True by default, meaning movement to waypoints is direct by default"""
        self.rc = robot_control.RobotControl()

        self.polar = polar

        # This is +y on our coordinate grid, should be the AUV's initial direction
        # pointed perpendicular to pool wall. Between 0 and 360 degrees from compass
        # heading! When we use the FOG, this will always be 0 degrees
        self.angle = 0 
        
        # Make the waypoints ordered pairs (x, y) and 
        # label w/ in-line comments. Always keep (0, 0)
        # as the first waypoint, measure all waypoints
        # relative to initial position. Units is meters.
        self.waypoints = [
            (0, 0) # Initial Position
            (1, 2) # example
            (3, 4) # second example
        ]
    
    def next(self):
        """Travels to the next waypoint, using the polar or rectilinear methods
        depending on the self.polar boolean variable"""
        if self.polar:
            self.polar_movement()
        else:
            self.rectilinear_movement()
    
    def rectilinear_movement(self):
        """Go to a waypoint using only forward and lateral motion (no yawing). May require
        compass or fiber-optic gyro but doesn't utilize it too consistently. May not be
        implemented since this is relatively primitive"""
        # Get x,y coordinates of top two waypoints
        x0, y0 = self.waypoints[0]
        x1, y1 = self.waypoints[1]
        
        # move laterally and forward based on change in x, y
        self.rc.set_heading(self.angle)
        self.rc.lateral_dvl(x1 - x0)
        self.rc.forward_dvl(y1 - y0)

        # Remove first waypoint element (since we've departed
        # from that place)
        self.waypoints.pop(0)
    
    def polar_movement(self):
        """Yaw towards waypoint then go directly. Requires compass or fiber-optic gyro."""
        # Get x,y coordinates of top two waypoints
        x0, y0 = self.waypoints[0]
        x1, y1 = self.waypoints[1]

        # Get delta-x, delta-y
        delta_x = x1 - x0
        delta_y = y1 - y0

        # Get required distance
        distance = math.sqrt((delta_x)**2 + (delta_y)**2)
        
        # Configure angles. Unlike usual math where 0 degrees is positive x-axis and counterclockwise
        # is positive, here the positive y-axis is 0 degrees and clockwise is positive

        # Get edge cases
        if delta_x == 0 and delta_y == 0:
            # Point at initial heading. If this condition is
            # triggering, you're implementing waypoints incorrectly
            angle = self.angle
            print("[WARN] Configured two waypoints at the same location")
        elif delta_x == 0:
            if delta_y > 0:
                angle = 0
            else:
                angle = 180
        elif delta_y == 0:
            if delta_x > 0:
                angle = 90
            else:
                angle = 270

        # Get quadrants
        if delta_x > 0 and delta_y > 0:
            angle = math.atan(abs(delta_x) / abs(delta_y))
        elif delta_x > 0 and delta_y < 0:
            angle = 90 + math.atan(abs(delta_y) / abs(delta_x))
        elif delta_x < 0 and delta_y < 0:
            angle = 180 + math.atan(abs(delta_x) / abs(delta_y))
        elif delta_x < 0 and delta_y > 0:
            angle = 270 + math.atan(abs(delta_y) / abs(delta_x))
        

        # Move to the waypoint
        self.rc.set_heading(angle)
        self.rc.forward_dvl(distance)

        # Remove first waypoint element (since we've departed
        # from that place)
        self.waypoints.pop(0)
