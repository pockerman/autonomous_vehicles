import time
from pi_camera_stream import setup_camera
from pi_camera_stream import start_stream
from pi_camera_stream import get_encoded_bytes_for_frame
from display_queue_utils import put_output_image

def get_control_instruction(control_queue):
    """
        Get control instructions from the web app, if any
    """

    if control_queue.empty():
        # nothing
        return None
    else:
        return control_queue.get()


def controlled_image_server_behavior(rotation, display_queue, control_queue):

    # Setup the camera
    camera = setup_camera(rotation=rotation)
    # allow the camera to warmup
    time.sleep(0.1)

    # Send frames from camera to server
    for frame in start_stream(camera):
        encoded_bytes = get_encoded_bytes_for_frame(frame)
        put_output_image(encoded_bytes, display_queue=display_queue)

        # Check any control instructions
        instruction = get_control_instruction(control_queue=control_queue)
        if instruction == "exit":
            print("Stopping")
            return
