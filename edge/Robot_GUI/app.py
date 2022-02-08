##template to this code was downloaded from https://github.com/miguelgrinberg/Flask-SocketIO

#!/usr/bin/env python
import sys
from threading import Lock
from flask import Flask, render_template, session, request, \
    copy_current_request_context
from flask_socketio import SocketIO, emit, join_room, leave_room, \
    close_room, rooms, disconnect
from SX127x.LoRa import *
from SX127x.LoRaArgumentParser import LoRaArgumentParser
from SX127x.board_config import BOARD
import RPi.GPIO as GPIO


BOARD.setup()

lora = LoRa()
lora.set_mode(MODE.STDBY)
lora.set_pa_config(pa_select=1)
lora.set_dio_mapping([1,0,0,0,0,0])
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)


parser = LoRaArgumentParser("A simple LoRa beacon")
parser.add_argument('--single', '-S', dest='single', default=False, action="store_true", help="Single transmission")
parser.add_argument('--wait', '-w', dest='wait', default=1, action="store", type=float, help="Waiting time between transmissions (default is 0s)")

args = parser.parse_args(lora)
#from lora.tx_command import LoRaBeacon

# Set this variable to "threading", "eventlet" or "gevent" to test the
# different async modes, or leave it set to None for the application to choose
# the best option based on installed packages.
async_mode = None

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, async_mode=async_mode)
thread = None
thread_lock = Lock()
#lora = LoRaBeacon(verbose=False)

def background_thread():
    #Example of how to send server generated events to clients.
    count = 0
    while True:
        socketio.sleep(10)
        count += 1
        socketio.emit('my_response',
                      {'data': 'Server generated event', 'count': count})


@app.route('/')
def index():
    return render_template('index.html', async_mode=socketio.async_mode)


@socketio.event
def drive_event(message):
    #lora.start(message)
    command = [message] 
    lora.write_payload(command)
    lora.set_mode(MODE.TX)
    GPIO.output(23, GPIO.HIGH)
    GPIO.output(24, GPIO.LOW)
    print(command)

@socketio.event
def send_message(message):
    print(message)
"""

@socketio.event
def rotate1_event(message):
    print(message)

@socketio.event
def join(message):
    join_room(message['room'])
    session['receive_count'] = session.get('receive_count', 0) + 1
    emit('my_response',
         {'data': 'In rooms: ' + ', '.join(rooms()),
          'count': session['receive_count']})


@socketio.event
def leave(message):
    leave_room(message['room'])
    session['receive_count'] = session.get('receive_count', 0) + 1
    emit('my_response',
         {'data': 'In rooms: ' + ', '.join(rooms()),
          'count': session['receive_count']})


@socketio.on('close_room')
def on_close_room(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    emit('my_response', {'data': 'Room ' + message['room'] + ' is closing.',
                         'count': session['receive_count']},
         to=message['room'])
    close_room(message['room'])


@socketio.event
def my_room_event(message):
    session['receive_count'] = session.get('receive_count', 0) + 1
    emit('my_response',
         {'data': message['data'], 'count': session['receive_count']},
         to=message['room'])
"""
@socketio.event
def disconnect_request():
    @copy_current_request_context
    def can_disconnect():
        disconnect()

    session['receive_count'] = session.get('receive_count', 0) + 1
    # for this emit we use a callback function
    # when the callback function is invoked we know that the message has been
    # received and it is safe to disconnect
    emit('my_response',
         {'data': 'Disconnected!', 'count': session['receive_count']},
         callback=can_disconnect)


@socketio.event
def my_ping():
    emit('my_pong')


@socketio.event
def connect():
    global thread
    with thread_lock:
        if thread is None:
            thread = socketio.start_background_task(background_thread)
    emit('my_response', {'data': 'Connected', 'count': 0})


@socketio.on('disconnect')
def test_disconnect():
    print('Client disconnected', request.sid)


if __name__ == '__main__':
    lora.set_freq(868)
    socketio.run(app)
    print(lora)
assert(lora.get_agc_auto_on() == 1)
BOARD.teardown()