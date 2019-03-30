"""
Handles the input from the camera module
"""

import cv2
import picamera



class Camera(object):

    def __init__(self):
        pass

    def capture(self):
        cv2.namedWindow("Camera", 1)
        capture = cv2.captureFromCAM(0)
        cv2.setCaptureProperty(capture, 3, 360)
        cv2.setCaptureProperty(capture, 4, 240)

        while True:
            img = cv2.QueryFrame(capture)
            cv2.showImage("Camera", img)

            if cv2.waitKey(10) == 27 :
                break
