## all mesaurements are in millimeters/pixels

focal_length = 4.81
real_pole_height = 914.4 # (3 ft)
image_height = 800 # (pixels)
object_height_pixels = input("Enter the height of the object in pixels: ")
camera_height = 54.5

def calculate_distance(object_height_pixels):
    """
    Calculate the distance to an object based on its height in pixels.
    """
    try:
        object_height_on_cam = float(object_height_pixels)
        if object_height_on_cam <= 0:
            raise ValueError("Object height must be a positive number.")
        
        distance = (focal_length * real_pole_height * image_height) / (object_height_on_cam * camera_height)
        return distance
    except ValueError as e:
        print(f"Invalid input: {e}")
        return None
    
print("distance from sub:", calculate_distance(object_height_pixels), "mm")