from multiprocessing import Process

from conf import USE_MULTIPROCESSING
from conf import INDEX_DISPLAY_TEMPLATE_NAME
from conf import CAMERA_SLEEP_TIME
from conf import DEBUG
from conf import HOST
from conf import PORT
from conf import NEEDED_CAMERA_ROTATION
from conf import display_queue
from conf import control_queue
from control_utils import controlled_image_server_behavior

from flask import Flask
from flask import render_template
from flask import Response

if USE_MULTIPROCESSING:
    from pi_camera_stream import frame_generator_from_queue
    CAMERA_SLEEP_TIME = 0.05
else:
    from pi_camera_stream import frame_generator

index_display_template = INDEX_DISPLAY_TEMPLATE_NAME


app = Flask(__name__)

@app.route('/')
def index():
    """
    Function that serves the index view
    """
    return render_template(index_display_template)


@app.route('/display')
def display():
    if USE_MULTIPROCESSING:
        return Response(frame_generator_from_queue(display_queue=display_queue, sleep_time=CAMERA_SLEEP_TIME),
                        mimetype='multipart/x-mixed-replace; boundary=frame')
    else:
        return  Response(frame_generator(rotation=NEEDED_CAMERA_ROTATION, sleep_time=CAMERA_SLEEP_TIME),
                         mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/control/<control_name>')
def control(control_name):
    control_queue.put(control_name)
    return Response('queued')

if USE_MULTIPROCESSING:

    def start_server_process(template_name):
        """
            Start the process, call .terminate to close it
        """
        global display_template
        display_template = template_name
        server = Process(target=app.run, kwargs={"host": HOST, "debug": DEBUG, "port": PORT})
        server.start()
        return server


    process = start_server_process('control_image_behavior.html')
    try:
        controlled_image_server_behavior(rotation=NEEDED_CAMERA_ROTATION, display_queue=display_queue, contorl_queue=control_queue)
    finally:
        process.terminate()

else:
    app.run(host=HOST, debug=DEBUG, port=PORT)
