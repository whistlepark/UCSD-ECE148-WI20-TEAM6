import cv2
import numpy as np


# input options in order (right->left): 'camera/rgb', 'camera/depth', 'slam/odom'

class ObjectDetection:
    # Look for area of depth image that correlates to closest object
    # Look for same location in rgb/infra
    # Detect most "present" object (i.e. the closest object that might be run into)
    # Return object location, height and width and pixel box location

    def __init__(self, rgb=None, depth=None):
        self.rgb = rgb
        self.depth = depth
        self.depth_color = None
        if rgb:
            self.height, self.width = rgb.shape[:2]
        else:
            self.height = None
            self.width = None
        self.depth_scale = 0.0010000000474974513
        self.closest = []
        self.area_thresh = 2000  # filter out objects less than 500 px^2

    def update_images(self, rgb, depth):
        self.rgb = np.asanyarray(rgb)
        self.depth = np.asanyarray(depth)
        self.height, self.width = rgb.shape[:2]
        self.depth_color = cv2.applyColorMap(cv2.convertScaleAbs(self.depth, alpha=0.03), cv2.COLORMAP_BONE)

    def update(self):
        if self.rgb is not None:
            self.visualize()

    def run_threaded(self, rgb, depth):
        if rgb is not None:
            self.update_images(rgb, depth)
            self.process()
            #self.visualize()
        return self.closest

    def process(self):
        self.closest = []
        # Perform processing on depth image: convert to cv image first
        self.depth = self.depth * self.depth_scale  # Convert values to float distance values
        imgray = cv2.cvtColor(self.depth_color, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(imgray, (5, 5), 0)
        thresh = cv2.threshold(blurred, 25, 255, cv2.THRESH_BINARY_INV)[1]
        kernel = np.ones((5, 5), np.uint8)
        closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)  # Erosion/Dilation and Closing Morphology
        contours, hierarchy = cv2.findContours(closing.copy(), cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            pass
        else:
            for i in range(len(contours)):
                cnt = contours[i]
                mask = np.zeros(imgray.shape, np.uint8)
                cv2.drawContours(mask, contours, i, 255, -1)
                # Get Moments and center location:
                area = cv2.contourArea(cnt)
                if area > self.area_thresh:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.rectangle(self.rgb, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cx = int(x + w / 2)
                    cy = int(y + h / 2)
                    cv2.circle(self.rgb, (cx, cy), 10, (255, 0, 0))
                    dist = cv2.mean(self.depth, mask=mask)[0]
                    self.closest.append(self.Object(cx, cy, dist, w, h))
                    text = 'Dist: ' + str(dist)
                    cv2.putText(self.rgb, text, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            # cv2.drawContours(self.rgb, cnt, -1, (0, 255, 0))

    def visualize(self):
        images = np.hstack((self.rgb, self.depth_color))
        cv2.namedWindow('RGB/DEPTH', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RGB/DEPTH', images)

    class Object:
        def __init__(self, cx=None, cy=None, dist=None, width=None, height=None):
            # Values are in pixels not meters... except for distance
            self.cx = cx
            self.cy = cy
            self.distance = dist
            self.width = width
            self.height = height

        def __lt__(self, other):
            return self.distance < other.distance

        def __gt__(self, other):
            return not self < other

        def update(self, cx, cy, distance, width, height):
            self.cx = cx
            self.cy = cy
            self.distance = distance
            self.width = width
            self.height = height
            return self
