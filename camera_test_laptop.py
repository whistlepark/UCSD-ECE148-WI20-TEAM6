import pyrealsense2 as rs
import cv2 as cv
import numpy as np


# Define Region of Interest
def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    if len(img.shape) > 2:
        channel_count = img.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255
    cv.fillPoly(mask, vertices, ignore_mask_color)
    masked_image = cv.bitwise_and(img, mask)
    return masked_image
