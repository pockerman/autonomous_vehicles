"""
Configuration parameters for Odisseus V2
"""

import cv2

DEBUG = True
PORT =  5001
HOST = '0.0.0.0'

# Flag indicating if we are on Pi or simply emulating
ON_RASP_PI:bool = False
SCREEN_SIZE:tuple = (320, 240)
ENCODE_PARAMS = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
INDEX_DISPLAY_TEMPLATE_NAME = 'image_server.html'
CAMERA_SLEEP_TIME = 0.05

# configuration for multiprocessing
USE_MULTIPROCESSING:bool = False
QUEUE_MAX_SIZE=2