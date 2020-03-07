import cv2  # state of the art computer vision algorithms library
import numpy as np  # fundamental package for scientific computing
import pyrealsense2 as rs  # Intel RealSense cross-platform open-source API

print("Environment Ready")

pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

try:
    while True:
        profile = pipe.start(cfg)

        # Skip 5 first frames to give the Auto-Exposure time to adjust
        for x in range(5):
            pipe.wait_for_frames()
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        print('Depth Scale: ', depth_scale)

        # Store next FRAMESET for later processing:
        frameset = pipe.wait_for_frames()
        color_frame = frameset.get_color_frame()

        # Cleanup:
        pipe.stop()
        print("Frames Captured")

        color = np.asanyarray(color_frame.get_data())

        align = rs.align(rs.stream.color)
        frameset = align.process(frameset)

        # Update color and depth frames:
        aligned_depth_frame = frameset.get_depth_frame()

        # Apply Post Processing Filters
        aligned_depth_frame = rs.hole_filling_filter().process(aligned_depth_frame)
        aligned_depth_frame = rs.spatial_filter().process(aligned_depth_frame)
        aligned_depth_frame = rs.temporal_filter().process(aligned_depth_frame)

except Exception as e:
    print(e)
