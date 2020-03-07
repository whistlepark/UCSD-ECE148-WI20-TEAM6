import cv2  # state of the art computer vision algorithms library
import numpy as np  # fundamental package for scientific computing
import matplotlib.pyplot as plt  # 2D plotting library producing publication quality figures
import pyrealsense2 as rs  # Intel RealSense cross-platform open-source API

print("Environment Ready")


# Define Region of Interest
def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    if len(img.shape) > 2:
        channel_count = img.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


# Setup:
thresh_val = 1  # meter...
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# scale = rs2_get_depth_scale(sensor, NULL)
# const uint16_t * image = (const uint16_t *)rs2_get_frame_data(frame, NULL)
# float depth_in_meters = 1 / (scale * image[pixel_index])

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

        # SET UP REGION OF INTEREST
        rows, cols = color.shape[:2]
        left_boundary = [int(cols * 0.40), int(rows * 0.95)]
        left_boundary_top = [int(cols * 0.40), int(rows * 0.20)]
        right_boundary = [int(cols * 0.60), int(rows * 0.95)]
        right_boundary_top = [int(cols * 0.60), int(rows * 0.20)]
        bottom_left = [int(cols * 0.20), int(rows * 0.95)]
        top_left = [int(cols * 0.20), int(rows * 0.20)]
        bottom_right = [int(cols * 0.80), int(rows * 0.95)]
        top_right = [int(cols * 0.80), int(rows * 0.20)]

        vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
        cv2.line(color, tuple(bottom_left), tuple(bottom_right), (255, 0, 0), 5)
        cv2.line(color, tuple(bottom_right), tuple(top_right), (255, 0, 0), 5)
        cv2.line(color, tuple(top_left), tuple(bottom_left), (255, 0, 0), 5)
        cv2.line(color, tuple(top_left), tuple(top_right), (255, 0, 0), 5)
        copied = np.copy(color)
        interested = region_of_interest(copied, vertices)
        cv2.line(color, tuple(left_boundary), tuple(left_boundary_top), (255, 0, 0), 5)
        cv2.line(color, tuple(right_boundary), tuple(right_boundary_top), (255, 0, 0), 5)

        # COLORIZE THE IMAGES
        colorizer = rs.colorizer()

        # Create alignment primitive with color as its target stream:
        align = rs.align(rs.stream.color)
        frameset = align.process(frameset)

        # Update color and depth frames:
        aligned_depth_frame = frameset.get_depth_frame()

        # Apply Post Processing Filters
        aligned_depth_frame = rs.hole_filling_filter().process(aligned_depth_frame)
        aligned_depth_frame = rs.spatial_filter().process(aligned_depth_frame)
        aligned_depth_frame = rs.temporal_filter().process(aligned_depth_frame)
        # dist = aligned_depth_frame.as_depth_frame().get_distance(146, 97)
        # print('Distance: ', dist)

        colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
        copied_depth = np.copy(colorized_depth)
        aligned_depth = np.asanyarray(aligned_depth_frame.get_data())

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(aligned_depth, alpha=0.03), cv2.COLORMAP_BONE)

        # edges = cv2.Canny(copied_depth, 100, 200)
        aligned_depth = aligned_depth * depth_scale

        interested_depth = region_of_interest(copied_depth, vertices)

        cv2.line(colorized_depth, tuple(bottom_left), tuple(bottom_right), (0, 255, 0), 5)
        cv2.line(colorized_depth, tuple(bottom_right), tuple(top_right), (0, 255, 0), 5)
        cv2.line(colorized_depth, tuple(top_left), tuple(bottom_left), (0, 255, 0), 5)
        cv2.line(colorized_depth, tuple(top_left), tuple(top_right), (0, 255, 0), 5)
        cv2.line(colorized_depth, tuple(left_boundary), tuple(left_boundary_top), (0, 255, 0), 5)
        cv2.line(colorized_depth, tuple(right_boundary), tuple(right_boundary_top), (0, 255, 0), 5)

        # Calculate mean depth in interested region of depth image:
        mask = np.zeros(colorized_depth.shape[:2], dtype=np.uint8)
        # top, left, bottom, right
        mask[top_left[1]:bottom_left[1], top_left[0]:top_right[0]] = 255
        colorized_depth = cv2.bitwise_and(colorized_depth, colorized_depth, mask=mask)
        # aligned_depth = cv2.bitwise_and(aligned_depth, aligned_depth, mask=mask)

        # Threshold for values less than one meter
        imgray = cv2.cvtColor(depth_colormap, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(imgray, (5, 5), 0)
        thresh = cv2.threshold(blurred, 25, 255, cv2.THRESH_BINARY_INV)[1]
        # thresh = cv2.bitwise_and(thresh, thresh, mask=mask)

        kernel = np.ones((5, 5), np.uint8)
        erosion = cv2.erode(thresh, kernel, iterations=5)
        dilate = cv2.dilate(erosion, kernel, iterations=5)

        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        # ret, imgray = cv2.threshold(imgray, 200, 255, cv2.THRESH_BINARY)
        contours_opening, hierarchy_opening = cv2.findContours(opening.copy(), cv2.RETR_EXTERNAL,
                                                               cv2.CHAIN_APPROX_SIMPLE)
        contours_closing, hierarchy_closing = cv2.findContours(closing.copy(), cv2.RETR_EXTERNAL,
                                                               cv2.CHAIN_APPROX_SIMPLE)
        contours_thresh, hierarchy_thresh = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_dilate, hierarchy_dilate = cv2.findContours(dilate.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours_closing
        heirarchy = hierarchy_closing

        if not contours:
            pass
        else:
            for i in range(len(contours)):
                cnt = contours[i]
                mask = np.zeros(imgray.shape, np.uint8)
                cv2.drawContours(mask, contours, i, 255, -1)
                # Get Moments and center location:
                M = cv2.moments(cnt)
                area = cv2.contourArea(cnt)
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(color, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cx = int(x + w / 2)
                cy = int(y + h / 2)
                cv2.circle(color, (cx, cy), 10, (255, 0, 0))
                # print('Object ', i, ' distance: ', cv2.mean(aligned_depth, mask=mask))
                text = 'Dist: ' + str(cv2.mean(aligned_depth, mask=mask)[0])
                try:
                    cv2.putText(color, text, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                except Exception as b:
                    print(b)

        cv2.drawContours(color, contours, -1, (0, 255, 0))
        test_images = np.hstack((thresh, opening, closing, dilate))
        cv2.namedWindow('TEST', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('TEST', test_images)

        # cv2.rectangle(mask, (inner_rect[0], inner_rect[1]), (inner_rect[2], inner_rect[3]), 0, -1)
        interested_depth = interested_depth * depth_scale
        # dist, _, _, _ = cv2.mean(colorized_depth[145, 97])
        # print('OpenCV Dist: ', dist)
        # pixel_distance_in_meters = depth_frame.get_distance(369, 42)
        # Show the two frames together:
        # cv2.bitwise_and(copied_depth, edges)
        # ret, thresh3 = cv2.threshold(colorized_depth, 127, 255, cv2.THRESH_TRUNC)
        # print(aligned_depth)
        images = np.hstack((color, depth_colormap))
        # Show images
        cv2.namedWindow('RealSense: (RGB, DEPTH-ALL_FILTERS)', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense: (RGB, DEPTH-ALL_FILTERS)', images)
        # cv2.imshow('Edges', edges)
        cv2.imshow('Aligned', aligned_depth)
        cv2.waitKey(1)

except Exception as e:
    print(e)
finally:
    pass
'''
# Standard OpenCV boilerplate for running the net:
height, width = color.shape[:2]
expected = 300
aspect = width / height
resized_image = cv2.resize(color, (round(expected * aspect), expected))
crop_start = round(expected * (aspect - 1) / 2)
crop_img = resized_image[0:expected, crop_start:crop_start+expected]

net = cv2.dnn.readNetFromCaffe("../MobileNetSSD_deploy.prototxt", "../MobileNetSSD_deploy.caffemodel")
inScaleFactor = 0.007843
meanVal       = 127.53
classNames = ("background", "aeroplane", "bicycle", "bird", "boat",
              "bottle", "bus", "car", "cat", "chair",
              "cow", "diningtable", "dog", "horse",
              "motorbike", "person", "pottedplant",
              "sheep", "sofa", "train", "tvmonitor")

blob = cv2.dnn.blobFromImage(crop_img, inScaleFactor, (expected, expected), meanVal, False)
net.setInput(blob, "data")
detections = net.forward("detection_out")

label = detections[0,0,0,1]
conf  = detections[0,0,0,2]
xmin  = detections[0,0,0,3]
ymin  = detections[0,0,0,4]
xmax  = detections[0,0,0,5]
ymax  = detections[0,0,0,6]

className = classNames[int(label)]

cv2.rectangle(crop_img, (int(xmin * expected), int(ymin * expected)),
             (int(xmax * expected), int(ymax * expected)), (255, 255, 255), 2)
cv2.putText(crop_img, className,
            (int(xmin * expected), int(ymin * expected) - 5),
            cv2.FONT_HERSHEY_COMPLEX, 0.5, (255,255,255))

plt.imshow(crop_img)


scale = height / expected
xmin_depth = int((xmin * expected + crop_start) * scale)
ymin_depth = int((ymin * expected) * scale)
xmax_depth = int((xmax * expected + crop_start) * scale)
ymax_depth = int((ymax * expected) * scale)
xmin_depth,ymin_depth,xmax_depth,ymax_depth
cv2.rectangle(colorized_depth, (xmin_depth, ymin_depth),
             (xmax_depth, ymax_depth), (255, 255, 255), 2)
plt.imshow(colorized_depth)

depth = np.asanyarray(aligned_depth_frame.get_data())
# Crop depth data:
depth = depth[xmin_depth:xmax_depth,ymin_depth:ymax_depth].astype(float)

# Get data scale from the device and convert to meters
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
depth = depth * depth_scale
dist,_,_,_ = cv2.mean(depth)
print("Detected a {0} {1:.3} meters away.".format(className, dist))

'''
