from .conf import ON_RASP_PI
from .conf import SCREEN_SIZE
from .conf import ENCODE_PARAMS

if ON_RASP_PI == True:
    from  picamera.array import PiRGBArray
    from  picamera import PiCamera
else:
    from .picam_mock import  PiRGBArray
    from .picam_mock import PiCamera


import cv2
import time

size = SCREEN_SIZE
encode_param = ENCODE_PARAMS

def setup_camera(rotation = 0.0)->PiCamera:
    """
    Set up the camera
    """

    camera = PiCamera()
    camera.resolution = size
    camera.rotation = rotation
    return camera


def start_stream(camera: PiCamera):

    image_storage = PiRGBArray(camera, size=size)

    # set up the stream of data. 'bgr' is the format OpenCV stores color data
    # use_video_port, which, when set to true, results in a reduction in
    # image quality in exchange for faster production of frames.

    cam_stream = camera.capture_continuous(image_storage=image_storage, format='bgr', use_video_port=True)

    for raw_frame in cam_stream:
        yield raw_frame.array

        # reset so that we can hold the next image
        image_storage.truncate(0)


def get_encoded_bytes_for_frame(frame)->str:
    """
    Encodes an image with OpenCV
    """

    result, encoded_img = cv2.imencode('.jpg', frame, encode_param)
    return encoded_img.tostring()

def frame_generator(rotation):
    """
    Main video feed
    """
    camera = setup_camera(rotation=rotation)

    # allow the camera to warm up
    time.sleep(0.1)

    for frame in start_stream(camera=camera):
        encode_bytes = get_encoded_bytes_for_frame(frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + encode_bytes + b'\r\n')