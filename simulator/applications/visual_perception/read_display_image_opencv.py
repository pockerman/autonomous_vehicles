"""
Read and display an image using OpenCV
"""
import cv2

def open_and_save(imgfile, imgoutfile):

	# load the image but in greyscale
	img = cv2.imread(imgfile)
	
	k = cv2.waitKey(0) & 0xFF
	
	if k==27: # wait for ESC key to exit
        cv2.destroyAllWindows()
    elif k == ord('s'): # wait for 's' key to save and exit
        cv2.imwrite(imgoutfile, img)
        cv2.destroyAllWindows()

	# the image is displayed in a window using
	#cv2.imshow('image', img)

	# need to wait otherwise we cannot see the image
	# we wait indefinitely
	#cv2.waitKey(0)


if __name__ == '__main__':
    imgfile = '../applications_data/visual_perception/images/messi5.jpg'
    imgoutfile ='../applications_data/visual_perception/images/messigray.png'
    open_and_save(imgfile=imgfile, imgoutfile=imgoutfile)
