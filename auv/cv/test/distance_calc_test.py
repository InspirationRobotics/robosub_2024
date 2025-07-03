

# these measurements are for the USD low-light camera

focal_length = 2.97 # 2.75 mm
real_pole_width = 25.4 # mm
image_width_px =  1433 # 4056 pixels
object_width_px = input("Enter the width of the object in pixels: ")
camera_width = 12.8 # 97 mm

def calculate_distance(object_width_px):
    """
    Calculate the distance to an object based on its width in pixels.
    """
    try:
        object_width_on_cam = float(object_width_px)
        if object_width_on_cam <= 0:
            raise ValueError("Object height must be a positive number.")
        
        distance = (focal_length * real_pole_width * image_width_px) / (object_width_on_cam * camera_width)
        return distance
    except ValueError as e:
        print(f"Invalid input: {e}")
        return None

print("distance from sub:", calculate_distance(object_width_px)/304.8, "ft")