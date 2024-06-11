"""Bin mission CV code"""

import time
import cv2
import numpy as np
import shapely

class CV: 
    """
    CV class for Bin mission. DO NOT change the name of the class, as this will mess up all of the backend files to run the CV scripts.
    """
  
    camera = "/auv/camera/videoOAKdRawBottom"
    model = "bins3"
    
    def __init__(self, **config):
        """Initialize CV class"""
      
        self.viz_frame = None
        self.error_buffer = []
      
        print("[INFO] Bin CV Init")

    def mask_out_lids(self, target, lids):
        """
        target: the target bin
        lids: the list of lids
        returns a new target bin with the lids masked out
        """

        if len(lids) == 0:
            return target, self.get_bbox_center(target)

        x1, x2, y1, y2 = target.xmin, target.xmax, target.ymin, target.ymax
        geom_target = shapely.geometry.box(x1, y1, x2, y2)

        for lid in lids:
            x1, x2, y1, y2 = lid.xmin - 100, lid.xmax + 100, lid.ymin, lid.ymax
            geom_lid = shapely.geometry.box(x1, y1, x2, y2)
            geom_target = geom_target.difference(geom_lid)

        # if there are multiple polygons, pick the largest one
        if geom_target.geom_type == "MultiPolygon":
            geom_target = max(geom_target, key=lambda x: x.area)

        # if there is a hole in the Polygon, split it into two
        if geom_target.geom_type == "Polygon" and geom_target.interiors:
            x1, x2, y1, y2 = geom_target.interiors[0].bounds
            split_line = shapely.geometry.LineString([(0, (y1 + y2) / 2), (640, (y1 + y2) / 2)])

            # split the polygon with the line
            splitted = shapely.ops.split(geom_target, split_line)

            # pick the largest polygon
            geom_target = max(splitted, key=lambda x: x.area)

        x1, y1, x2, y2 = geom_target.bounds
        target.xmin = x1
        target.ymin = y1
        target.xmax = x2
        target.ymax = y2

        # get the centroid
        centroid = geom_target.centroid
        target_center = (int(centroid.x), int(centroid.y))
        return target, target_center

    def get_bbox_center(self, detection):
        x1 = int(detection.xmin)
        x2 = int(detection.xmax)
        y1 = int(detection.ymin)
        y2 = int(detection.ymax)

        return ((x1 + x2) // 2, (y1 + y2) // 2)

    def run(self, frame, target, oakd_data):
        """
        Run the CV script.

        Args:
            frame: The frame from the camera stream
            target: This can be any type of information, for example, the object to look for
            detections: This only applies to OAK-D cameras; this is the list of detections from the ML model output

        Here should be all the code required to run the CV.
        This could be a loop, grabbing frames using ROS, etc.

        Returns:
            dictionary, visualized frame: {motion commands/flags for servos and other indication flags}, visualized frame
        """

        forward = 0
        lateral = 0
        aligned = False

        height, width, _ = frame.shape

        tolerance = 0.1
        maxConfidence = 0
        target_bin = None
        earth = None
        abydos = None
        earth_confidence = 0
        abydos_confidence = 0
        bins = []
        lids = []

        # measured offset from the footage
        target_pixel = (190, 300)

        if len(oakd_data) == 0:
            return {"forward": 0.8}, frame

        for detection in oakd_data:
            x1 = int(detection.xmin)
            x2 = int(detection.xmax)
            y1 = int(detection.ymin)
            y2 = int(detection.ymax)

            if "abydos" in detection.label and detection.confidence > abydos_confidence:
                abydos = detection
                abydos_confidence = detection.confidence

            elif "earth" in detection.label and detection.confidence > earth_confidence:
                earth = detection
                earth_confidence = detection.confidence

            elif "bin" in detection.label:
                bins.append(detection)

            elif "lid" in detection.label:
                lids.append(detection)

        if "earth" in target:
            target_bin = earth

        elif "abydos" in target:
            target_bin = abydos

        approach = False
        if target_bin is None:
            if earth is None and abydos is None:
                if len(bins) == 0:
                    return {"forward": 0.8}, frame
                else:
                    # average the positions of the bins
                    avg_center = np.mean([self.get_bbox_center(b) for b in bins], axis=0)
                    target_bin_center = (int(avg_center[0]), int(avg_center[1]))
                    approach = True
            elif earth is None:
                target_bin = abydos
            elif abydos is None:
                target_bin = earth

        if not approach:
            # remove the lids from the target bin
            target_bin, target_bin_center = self.mask_out_lids(target_bin, lids)

        cv2.rectangle(
            frame,
            (int(target_bin.xmin), int(target_bin.ymin)),
            (int(target_bin.xmax), int(target_bin.ymax)),
            (255, 0, 0),
            2,
        )

        cv2.circle(frame, target_pixel, 10, (0, 255, 0), -1)
        cv2.circle(frame, target_bin_center, 10, (0, 0, 255), -1)

        x_error = (target_bin_center[0] - target_pixel[0]) / width
        y_error = (target_pixel[1] - target_bin_center[1]) / height

        # apply a gain and clip the values
        lateral = np.clip(x_error * 3.5, -1, 1)
        forward = np.clip(y_error * 3.5, -1, 1)

        if len(self.error_buffer) > 30:
            self.error_buffer.pop(0)
        self.error_buffer.append((x_error + y_error) / 2)
        avg_error = np.mean(np.linalg.norm(self.error_buffer, axis=1))

        if avg_error < tolerance and len(self.error_buffer) == 30:
            aligned = True

        # TODO (low priority): Remove Lids for each bin
        return {"lateral": lateral, "forward": forward, "aligned": aligned}, frame

if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.cv.template_cv"
    
    # Create a CV object with arguments
    cv = CV()
    
    # here you can for example initialize your camera, etc
    cap = cv2.VideoCapture("../../testing_data/")

    while True:
        # grab a frame
        ret, frame = cap.read()
        if not ret:
            break

        # run the cv
        result = cv.run(frame, "some_info", None)
    
        # do something with the result
        print(f"[INFO] {result}")
    
        # debug the frame
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
