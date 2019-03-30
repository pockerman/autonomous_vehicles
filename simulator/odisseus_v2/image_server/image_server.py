from flask import Flask
from flask import render_template
from flask import Response

from .pi_camera_stream import frame_generator


app = Flask(__name__)

@app.route('/')
def index():
    return render_template('image_server.html')


@app.route('/display')
def display():
    return  Response(frame_generator(rotation=0.0), mimetype='multipart/x-mixed-replace; boundary=frame')


app.run(host='0.0.0.0', debug=True, port=5001)
