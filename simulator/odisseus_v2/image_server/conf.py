"""
Configuration parameters for Odisseus V2
"""

import cv2

ON_RASP_PI:bool = False
SCREEN_SIZE:tuple = (320, 240)
ENCODE_PARAMS = [int(cv2.IMWRITE_JPEG_QUALITY), 90]