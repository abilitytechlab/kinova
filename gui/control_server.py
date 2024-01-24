import sys
import time
import cv2
from flask import Flask, render_template, Response
from flask_socketio import SocketIO
import roslibpy
# The 2 in the next line describes the camera index, cv2.CAP_V4L2 was needed
# for my camera but might not be needed for another
videoCapture = cv2.VideoCapture(2, cv2.CAP_V4L2)
videoCapture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
videoCapture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

def generate_frames():
    while True:
        returnValue, image = videoCapture.read()
        if returnValue:
            ret, buffer = cv2.imencode('.jpg', image)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            print("Error reading video feed")
            break

class ControlServer():
    def __init__(self):
        self.app = Flask(__name__)
        self.app.route('/')(lambda: render_template('interface.html'))
        self.app.route('/video_feed')(self.video_feed)
        self.socketio = SocketIO(self.app)
        self.ros = roslibpy.Ros(host='0.0.0.0', port=9090)

        self.publisher = roslibpy.Topic(self.ros, '/eye_control/in', 'eye_control/guiMsg')
        self.service_request = roslibpy.ServiceRequest()
        self.on_state = True
        self.turn_off = roslibpy.Service(self.ros, 'j2n6s300_driver/in/stop', 'kinova_msgs/stop')
        self.turn_on = roslibpy.Service(self.ros, 'j2n6s300_driver/in/start', 'kinova_msgs/start')
        self.home = roslibpy.Service(self.ros, 'j2n6s300_driver/in/home_arm', 'kinova_msgs/HomeArm')
        self.changeFingerState = roslibpy.Service(self.ros, 'eye_control/change_finger_state', 'eye_control/changeFingerState')
        self.message_template = roslibpy.Message({'twist_linear_x': 0.0,
                                         'twist_linear_y': 0.0,
                                         'twist_linear_z': 0.0,
                                         'twist_angular_x': 0.0,
                                         'twist_angular_y': 0.0,
                                         'twist_angular_z': 0.0,})
        self.speed = 2.0
        # The following two arrays are used to more compactly determine the 
        # content of the move message
        self.positive_directions = {'left': 'twist_linear_x',
                                    'back': 'twist_linear_y',
                                    'up': 'twist_linear_z',
                                    'pitchLeft': 'twist_angular_x',
                                    'rollLeft': 'twist_angular_y',
                                    'spinRight': 'twist_angular_z'}
        self.negative_directions = {'right': 'twist_linear_x',
                                    'forward': 'twist_linear_y',
                                    'down': 'twist_linear_z',
                                    'pitchRight': 'twist_angular_x',
                                    'rollRight': 'twist_angular_y',
                                    'spinLeft': 'twist_angular_z'}

    def run(self):
        self._register_socketio_handlers()
        # self.app.run()
        self.ros.run()
        self.socketio.run(self.app, debug=False, host='0.0.0.0', port=8000)

    def move(self, direction):
        i = 1
        while i < 100:
            print(direction, file=sys.stderr)
            message = self.message_template.copy()
            if direction in self.positive_directions:
                message[self.positive_directions[direction]] = self.speed
            elif direction in self.negative_directions:
                message[self.negative_directions[direction]] = -self.speed
            self.publisher.publish(message)
            time.sleep(0.01)
            i += 1

    def video_feed(self):
        return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

    def _register_socketio_handlers(self):

        @self.socketio.on('onOff')
        def onOff():
            if self.on_state:
                self.turn_off.call(self.service_request)
                self.on_state = False
            else:
                self.turn_on.call(self.service_request)
                self.on_state = True

        @self.socketio.on('home')
        def home():
            print('Calling home service...')
            result = self.home.call(self.service_request)
            print(result['homearm_result'])

        @self.socketio.on('close')
        def close_hand():
            request = roslibpy.ServiceRequest({'state': False})
            self.changeFingerState.call(request)

        @self.socketio.on('open')
        def open_hand():
            request = roslibpy.ServiceRequest({'state': True})
            self.changeFingerState.call(request)

        # The rest of the document should probably be simplified but I am not
        # sure how to do so, this works for now
        @self.socketio.on('left')
        def left():
            self.move('left')            

        @self.socketio.on('right')
        def right():
            self.move('right')

        @self.socketio.on('forward')
        def forward():
            self.move('forward')

        @self.socketio.on('back')
        def back():
            self.move('back')

        @self.socketio.on('up')
        def up():
            self.move('up')

        @self.socketio.on('down')
        def down():
            self.move('down')

        @self.socketio.on('pitchLeft')
        def pitch_left():
            self.move('pitchLeft')

        @self.socketio.on('pitchRight')
        def pitch_right():
            self.move('pitchRight')

        @self.socketio.on('rollRight')
        def roll_right():
            self.move('rollRight')

        @self.socketio.on('rollLeft')
        def roll_left():
            self.move('rollLeft')

        @self.socketio.on('spinRight')
        def spin_right():
            self.move('spinRight')

        @self.socketio.on('spinLeft')
        def spin_left():
            self.move('spinLeft')
