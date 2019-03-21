"""
This tutorial will walk you through the basic utilities
that OpenCV has for video manipulation. It is taken from
https://docs.opencv.org/3.4.2/dd/d43/tutorial_py_video_display.html
"""
import numpy as np
import cv2 as cv


# to capture a video we need to create a VideoCapture object.
# The argument can be either
# a device index
# the name of a video file

cap = cv.VideoCapture(0)

while(True):

    # capture frame-by-frame
    ret, frame = cap.read()

    # operations on the captured frame
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

    # display the resulting frame
    cv.imshow('frame', gray)

    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# when everything is done release the capture
cap.release()
cv.destroyAllWindows()


# let's now see how we can save a video