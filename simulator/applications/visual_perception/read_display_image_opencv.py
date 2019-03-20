"""
In this application tutorial, we will be using OpenCV to read and display an image.
Read and display an image using OpenCV
"""
import cv2

# load the image but in greyscale
img = cv2.imread('../applications_data/visual_perception/images/messi5.jpg')

# the image is displayed in a window using
cv2.imshow('image', img)

# need to wait otherwise we cannot see the image
# we wait indefinitely
cv2.waitKey(0)

# destroy all the windows
cv2.destroyAllWindows()

# if we want to destroy a specific window we can
# cv2.destroyWindow()

# saving an image is done via cv2.imwrite()
cv2.imwrite('..../applications_data/visual_perception/images/messigray.png', img)