import cv2  #Not really needed
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

        self.align = rs.align(rs.stream.color)
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

    # Apply filters to depth frame if desired
    def apply_filters(self, depth_frame):
        depth_frame = self.spatial_filter.process(depth_frame)
        depth_frame = self.temporal_filter.process(depth_frame)
        return depth_frame

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        if not depth_frame or not color_frame:
            return False, None, None

        depth_image_mm = depth_image * self.depth_scale * 1000  # returns depth in mm

        return True, depth_image_mm, color_image

    def release(self):
        self.pipeline.stop()


# Initializes the camera object and captures a X images
def get_image(camera, num_images):
    images = []
    depth_frames = []
    
    # Warm up the camera to prevent dark frames
    warm_up_time = 5  # Warmup time in seconds
    start_time = time.time()

    while time.time() - start_time < warm_up_time:
        ret, depth_frame, frame = camera.get_frame()
        if not ret:
            print("Error: Could not read from the camera.")
            camera.release()
            exit()

    for _ in range(num_images):  # Take as many images as defined
        _, depth_frame, frame = camera.get_frame() 
        images.append(frame)
        depth_frames.append(depth_frame)
        time.sleep(0.1) # Delay between images (probably need to change it)
    camera.release()

    return depth_frames, images


def compare_bounding_boxes(all_center_points, num_to_appear=5, tolerance=20):
    # all_center_points is a list of lists containing center points from each image
    point_counts = {}
    for centers in all_center_points:
        for item in centers:
            center = item['center']
            key = (center[0] // tolerance, center[1] // tolerance)  # Determine if in bounds of tolerance
            if key in point_counts:
                point_counts[key]['count'] += 1
                point_counts[key]['centers'].append(center)
            else:
                point_counts[key] = {'count': 1, 'centers': [center]}

    # Keep centers that appear in multiple images
    consistent_centers = []
    for value in point_counts.values():
        if value['count'] >= num_to_appear:  # How many times a coordinate has to appear for it to be saved
            # Average the centers
            avg_center_x = sum(c[0] for c in value['centers']) / value['count']   #Take average centers so the value stays consistent
            avg_center_y = sum(c[1] for c in value['centers']) / value['count']
            consistent_centers.append((int(avg_center_x), int(avg_center_y)))
    return consistent_centers


def closest_to_center(centers, image_center):
    if not centers:
        return None
    closest_center = min(centers, key=lambda c: (c[0] - image_center[0]) ** 2 + (c[1] - image_center[1]) ** 2)
    return closest_center

# Gets the coordinates of the bounding box closest to the image center
def get_coordinates(results): 
    center_points = []
    boxes = results[0].boxes
    for box in boxes:
        x_min, y_min, x_max, y_max = map(int, box.xyxy[0])  # Get coordinates of each corner
        center_x = (x_max + x_min) // 2
        center_y = (y_max + y_min) // 2
        center_points.append({'center': (center_x, center_y), 'box': (x_min, y_min, x_max, y_max)})
    return center_points

# D400 Cameras depth is automatically in mm from DepthCamera Class
def get_depth(point, depth_frame, unit="mm"): 

    #height, width = depth_frame.shape[:2]
    #center_x = width // 2
    #center_y = height // 2
    
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


def draw_bounding(pixel, coordinates, image):
    rect_width = 120
    rect_height = 100

    top_left_x = int(pixel[0] - rect_width / 2)
    top_left_y = int(pixel[1] - rect_height / 2)
    bottom_right_x = int(pixel[0] + rect_width / 2)
    bottom_right_y = int(pixel[1] + rect_height / 2)


    cv2.rectangle(image, (top_left_x, top_left_y), (bottom_right_x, bottom_right_y), color=(255, 0, 0), thickness=2)

    cv2.circle(image, (pixel[0], pixel[1]), radius=5, color=(255, 0, 0), thickness=-1)

    text = f'({coordinates[0]}mm, {coordinates[1]}mm, {coordinates[2]}mm)'


    (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)

    text_x = 10
    text_y = 10 + text_height

    cv2.putText(image, text, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color=(0, 0, 255), thickness=1)
    
    cv2.imshow('Captured Image', image)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

def main():
    model = YOLO('foduucom/plant-leaf-detection-and-classification')

    # Initialize camera
    dc = DepthCamera()


    # Set model parameters
    X = 10  # Maximum number of leaves to detect per image
    N = 50  # Number of images to capture
    model.overrides['conf'] = 0.8  # Confidence threshold was 8
    model.overrides['iou'] = 0.45  # IoU threshold
    model.overrides['agnostic_nms'] = False  # Class-agnostic NMS
    model.overrides['max_det'] = X  # Max detections per image was 5

    # Capture multiple images
    depth_frames, images = get_image(dc, N)
    depth_display = cv2.applyColorMap(cv2.convertScaleAbs(depth_frames[0], alpha=0.5), cv2.COLORMAP_JET)


    image_path = 'captured_image.jpg'
    cv2.imwrite(image_path, images[0])
    display_image = cv2.imread(image_path)

    # Get image dimensions and center
    height, width = images[0].shape[:2]
    image_center = (width // 2, height // 2)

    # For each image, get the bounding boxes
    all_center_points = []
    for frame in images:
        # Perform model inference directly on the frame
        results = model.predict(frame)
        centers = get_coordinates(results)
        all_center_points.append(centers)

    # Compare the bounding boxes across images
    consistent_centers = compare_bounding_boxes(all_center_points, N//3, tolerance=10)

    # Select the leaf closest to the center
    leaf_center = closest_to_center(consistent_centers, image_center)

    if leaf_center is not None:
        # Use the depth frame corresponding to the first image (adjust as needed)
        depth = get_depth(leaf_center, depth_frames[0])

        if depth is not None:

            depth_mm = int(depth)
            x_mm, y_mm = pixel_to_mm(leaf_center[0], leaf_center[1], depth_mm, dc.intrinsics)
            position = (x_mm, y_mm, depth_mm)
            draw_bounding(leaf_center, position, display_image)

            cv2.imshow("Depth Image", depth_display)  # Just for display purposes
            cv2.waitKey(0)                            # Just for display purposes
            cv2.destroyAllWindows                     # Just for display purposes
            #print(f"Leaf position (mm): {position}")
            return position
        else:
            return None
    else:
        return None

if __name__ == "__main__":
    main()
