"""
introductory example on matplotlib
"""

from PIL import Image
from pylab import *

# read image
pil_img = Image.open('../images/pil_img1.jpeg')

# show the image
imshow(pil_img)

# introduce some points
x = [100, 100, 400, 400]
y = [200, 500, 200, 500]

# plot the points with red star-markers
plot(x,y,'r*')

# line plot connecting the first two points
plot(x[:2],y[:2])

# add title and show the plot
title('Plotting: "test.jpg"')

show()