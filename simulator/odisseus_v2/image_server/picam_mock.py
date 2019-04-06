
import cv2

class PiRGBArray(object):

    def __init__(self, camera, size):
        self.__camera = camera
        self.__size = size

    def truncate(self, val):
        pass


class ImgArray(object):

    def __init__(self, img):
        self.__img = img

    @property
    def array(self):
        return self.__img

    @array.setter
    def array(self, value):
        self.__img = value

class PiCamera(object):
    """
    Mock the PiCamera class
    """

    def __init__(self):
        self.__resolution = None
        self.__rotation = None
        self.__img = None


    @property
    def resolution(self):
        return self.__resolution

    @resolution.setter
    def resolution(self, value):
        self.__resolution = value

    @property
    def rotation(self):
        return self.__rotation

    @rotation.setter
    def rotation(self, value):
        self.__rotation = value


    def capture_continuous(self, image_storage, format, use_video_port):

        """
        Continuouisly send the same image
        """

        if self.__img is None:
            self.__img = cv2.imread('img/image_1.png')

        return [ImgArray(self.__img)]*10