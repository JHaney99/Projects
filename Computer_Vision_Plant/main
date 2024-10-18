import cv2
from ultralyticsplus import YOLO
import numpy as np
import pyrealsense2 as rs
import time


#Sets up the realsense camera
class DepthCamera:
    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        depth_sensor = device.first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale() ######## depth * this returns the depth in meters


        # Options for the device properties (framerate/resolution). For larger list, use Camera_Settings.py
        #config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) 
        #config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 5) # Highest resolution, lowest framerate - This causes the runtime to increase a good amount
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 5)
        

        # Start streaming
        self.profile = self.pipeline.start(config)
        
        # Initialize post-processing filters for later use if desired, I originally added them because depth was giving me issues and I wanted to see if this would fix it
        self.spatial_filter = rs.spatial_filter()  # Improves depth noise reduction
        self.temporal_filter = rs.temporal_filter()  # Reduces temporal noise

        # Get color sensor's intrinsics so I can later determine X, Y
        color_stream = self.profile.get_stream(rs.stream.color)
        self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

    # Apply filters to depth frame if necessary #######################
    def apply_filters(self, depth_frame):
        depth_frame = self.spatial_filter.process(depth_frame)
        depth_frame = self.temporal_filter.process(depth_frame)
        return depth_frame

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

       # depth_frame = self.apply_filters(depth_frame) #######################

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None
        
        depth_image_mm = depth_image * self.depth_scale * 1000  # returns depth in mm

        return True, depth_image_mm, color_image

    def release(self):
        self.pipeline.stop()


# Initializes the camera object and captures a single image
def get_image(camera):
    # Warm up the camera to prevent dark frames
    warm_up_time = 5  # Set the warm-up time (seconds)
    start_time = time.time()

    while time.time() - start_time < warm_up_time:
        ret, depth_frame, frame = camera.get_frame()
        if not ret:
            print("Error: Could not read from the camera.")
            camera.release()
            exit()

    # Capture the final frame after the warm-up period
    _, depth_frame, frame = camera.get_frame() # ret not needed, if it failed in the while loop software would exit()

    # Release the camera
    camera.release()

    return depth_frame, frame

#For this I think I need to assume that the results has been trimmed down to a single box that I am interested in. This can be modified later
def get_coordinates(results): 
    #center_points = []
    boxes = results[0].boxes
    for box in boxes:
        x_min, y_min, x_max, y_max = map(int, box.xyxy[0]) # Gets the coordinates of each corner of the box
        center_x = (x_max + x_min) // 2
        center_y  = (y_max + y_min) // 2

        #center_points.append((center_x, center_y))
        center_points = (center_x, center_y)   
        return center_points

# D400 Cameras depth is automatically in mm from DepthCamera Class
def get_depth(point, depth_frame, unit="mm"): 
    distance = depth_frame[point[1], point[0]]
   #Check if depth is out of range
    if distance == 0:
        distance = None
        return distance
    
    if unit == "cm":
        distance /= 10

    return distance

#Should convert the pixel coordinates to real world coordinates in mm. If I want to convert to cm I just divide values by 10
def pixel_to_mm(pixel_x, pixel_y, depth, intrinsics, unit="mm"):
    x = (pixel_x - intrinsics.ppx) / intrinsics.fx * depth
    y = (pixel_y - intrinsics.ppy) / intrinsics.fy * depth

    if unit == "cm":
        x /= 10
        y /= 10
    return x, y

# Main function where the program runs. It should be noted that some of the code being ran should be removed in the final version. The parts where images are rendered will not be necessary in the final version. They are for debugging purposes. Additionally, I have strings printing for my own understanding. Things such as "No Leaf Detected" should just be a bool False or None
def main():
    # Load YOLO model
    model = YOLO('foduucom/plant-leaf-detection-and-classification')

    # Initialize camera
    dc = DepthCamera()

    # Set model parameters (Note, conf doesnt seem to do anything, I can set a conf value manually in predict function conf=0.8)
    model.overrides['conf'] = 0.8  # NMS confidence threshold
    model.overrides['iou'] = 0.45  # NMS IoU threshold
    model.overrides['agnostic_nms'] = False  # NMS class-agnostic
    model.overrides['max_det'] = 1 # Maximum number of detections per image

    depth_frame, frame = get_image(dc)

    # Save the captured image
    image = 'captured_image.jpg'
    cv2.imwrite(image, frame)

    # Model inference via YOLO
    results = model.predict(image)

    # Process results
    if len(results[0].boxes) > 0:
        leaf_center = get_coordinates(results)
        depth = get_depth(leaf_center, depth_frame)
        
        if depth != None:
            depth_mm = int(depth)  # Convert depth to integer (May have to keep as float if arm requires that high of precision, however, depth value alone will be off by a bit so I dont think that will matter)
            x_mm, y_mm = pixel_to_mm(leaf_center[0], leaf_center[1], depth_mm, dc.intrinsics)
            position = (x_mm, y_mm, depth_mm)
            #return position
            print(position)
        else:
            print("Depth is out of range, coordinates cannot be obtained")
            # Recursive call main itself, retake photo?
            # main()
    else:
        print("No leaf detected, exiting")
        #Recursive call main itself? retake photo?
        exit()

if __name__ == "__main__":
    main()
