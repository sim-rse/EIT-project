""" IMPORTS """

import time
import pyrealsense2 as rs
import cv2 as cv2
import numpy as np
from threading import Thread, Lock
from dataclasses import dataclass
import datetime
    
    
""" FUNCTIONS """

def intrinsics_to_numpy(intrinsics: rs.intrinsics):
    """ Converts realsense intrinsics into numpy arrays """

    camera_matrix = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                              [0, intrinsics.fy, intrinsics.ppy],
                              [0, 0, 1]])

    distortion = np.array(intrinsics.coeffs)

    (h, w) = np.array([intrinsics.height, intrinsics.width])

    return camera_matrix, distortion


""" CLASSES """

@dataclass
class RealSenseFrame:

    def __init__(self, color_map=None, depth_map=None, left_ir_map=None, right_ir_map=None):
        self.time_stamp = datetime.datetime.utcnow()
        self.color_map = color_map
        self.depth_map = depth_map
        self.left_ir_map = left_ir_map
        self.right_ir_map = right_ir_map


class RealSenseCamera:

    def __init__(self, on_update=None):

        # Create the pipeline (handles all connected realsense devices) and device config object (optional)
        self.__pipeline = rs.pipeline()
        self.__config = rs.config()
        
        realsense_ctx = rs.context()
        connected_devices = []
        for i in range(len(realsense_ctx.devices)):
            detected_camera = realsense_ctx.devices[i].get_info(rs.camera_info.serial_number)
            connected_devices.append(detected_camera)

        # Enable depth and color sensors
        self.__config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)   # Depth map uint16
        self.__config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Color map uint8 (x3 channels)

        # Configure depth frame post processing pipeline
        self.__depth_filters = [rs.decimation_filter(magnitude=1),
                                rs.threshold_filter(min_dist=0.3, max_dist=4),
                                rs.disparity_transform(transform_to_disparity=True),
                                rs.spatial_filter(smooth_alpha=0.5, smooth_delta=20.0, magnitude=2.0, hole_fill=2),
                                rs.temporal_filter(smooth_alpha=0.4, smooth_delta=20.0, persistence_control=3),
                                #rs.hole_filling_filter(mode=2),
                                rs.disparity_transform(transform_to_disparity=False)]

        self.__align = rs.align(rs.stream.color)   # Align object which aligns depth to color frames
        self.__kill = False                        # Flag to kill background process
        self.__thread = Thread()                   # Background threading object
        self.__mutex = Lock()

        # Properties
        self.__latest_frame = RealSenseFrame()
        self.__prev_timestamp = self.__latest_frame.time_stamp

        self.__depth_frame = None
        self.__color_frame = None
        self.__left_ir_frame = None
        self.__right_ir_frame = None

        # Set optional callback method
        if on_update is None or callable(on_update):
            self.__callback = on_update
        else:
            raise TypeError("callback function must be callable")

    def start_async(self):

        if self.__thread.is_alive():
            print(f"An existing process is running: id={self.__thread.native_id}")
            return

        self.__kill = False
        self.__thread = Thread(target=self.__camera_thread, daemon=True)
        self.__thread.start()

    def stop_async(self):

        if self.__thread.is_alive():
            self.__kill = True
            self.__thread.join()

    def __camera_thread(self):

        # Start streaming from realsense camera
        profile = self.__pipeline.start(self.__config)

        # Some RealSense cameras report distance values in odd units (1/32 meter)
        # Get the scale of the depth camera --> Scales distance to Meters --> Scale to Millimeters
        self.__depth_scale = profile.get_device().first_depth_sensor().get_depth_scale() * 1000

        try:
            while not self.__kill:

                # Capture temporal synchronized device data
                frames = self.__pipeline.wait_for_frames()

                # Align the depth frame to color frame
                aligned_frames = self.__align.process(frames)

                # Get the needed frames form the frames pipeline
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                # Apply filters to depth frame
                for f in self.__depth_filters:
                    depth_frame = f.process(depth_frame)

                # Update frame references
                self.__color_frame = color_frame
                self.__depth_frame = depth_frame

                # Create a numpy array images
                # A numpy array can be constructed using this protocol with no data marshalling overhead:
                depth_map = np.asanyarray(depth_frame.get_data())
                color_map = np.asanyarray(color_frame.get_data())

                # Results object
                data = RealSenseFrame(color_map, depth_map)

                # Callback
                if callable(self.__callback):
                    self.__callback(data)

                # Update latest data
                with self.__mutex:
                    self.__latest_frame = data

        finally:
            self.__pipeline.stop()
            cv2.destroyAllWindows()

    def get_latest_frame(self, copy=True):

        # Black until new frame
        while self.__latest_frame.time_stamp is self.__prev_timestamp:
            time.sleep(0.006)

        # Update ts
        with self.__mutex:
            self.__prev_timestamp = self.__latest_frame.time_stamp
            data = self.__latest_frame

        return data

    @property
    def depth_scale(self):
        if self.__thread.is_alive():
            return self.__pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale() * 1000
        else:
            return 0  # If the current depth sensor is offline, return 0 to make it obvious

    @property
    def is_alive(self):
        return self.__thread.is_alive()


    def unproject_pt_to_xyz(self, u, v, dist, sensor="color"):

        intrinsics = self.__get_intrinsics(sensor)
        pt = rs.rs2_deproject_pixel_to_point(intrinsics, [u, v], dist)
        return np.array([pt[0], pt[1], pt[2]])


    def project_xyz_to_pt(self, x, y, z, sensor):

        intrinsics = self.__get_intrinsics(sensor)
        pt = rs.rs2_project_point_to_pixel(intrinsics, [x, y, z])
        return pt

    @property
    def depth_intrinsics(self):

        intrinsics = rs.video_stream_profile(self.__depth_frame.profile).get_intrinsics()
        return intrinsics_to_numpy(intrinsics)

    @property
    def color_intrinsics(self):

        intrinsics = rs.video_stream_profile(self.__color_frame.profile).get_intrinsics()
        return intrinsics_to_numpy(intrinsics)

    def __get_intrinsics(self, sensor):

        if sensor == "color":
            frame = self.__color_frame
        elif sensor == "depth":
            frame = self.__depth_frame
        else:
            raise NotImplementedError("selected sensor intrinsics are not implemented")

        # Grab new intrinsics (may be changed by decimation)
        return rs.video_stream_profile(frame.profile).get_intrinsics()


""" MAIN """

if __name__ == "__main__":

    devices = rs.context().query_devices()
    print("Devices")
    for device in devices:
        print(device)

    # Instantiate the realsense camera and start
    camera = RealSenseCamera()
    camera.start_async()
    
    devices = rs.context().query_devices()
    for device in devices:
        print(device)
    
    # Main loop
    while camera.is_alive:

        # Get the latest image frame (this is blocking!)
        frame = camera.get_latest_frame()

        # Sensor maps may be accessed
        color_image = frame.color_map
        depth_image = frame.depth_map

        # Do something with color and depth maps

        # View the processed sensor data
        depth_image = cv2.convertScaleAbs(depth_image, alpha=0.03)               # Scale depth from 16 to 8 bit
        depth_image = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)  # Normalize map
        depth_image = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)           # Apply colormap
        cv2.imshow("Stream", np.hstack((color_image, depth_image)))              # Display color and depth maps together
        cv2.waitKey(1)                                                           # Wait 1 millisecond