##template to this code was downloaded from https://github.com/miguelgrinberg/Flask-SocketIO

#!/usr/bin/env python
import sys
import time
import struct
from threading import Lock, Thread
from queue import Queue
from flask import Flask, render_template, session, request, \
    copy_current_request_context
from flask_socketio import SocketIO, emit, join_room, leave_room, \
    close_room, rooms, disconnect
from SX127x.LoRa import *
from SX127x.LoRaArgumentParser import LoRaArgumentParser
from SX127x.board_config import BOARD
import RPi.GPIO as GPIO

app = Flask(__name__)
socketio = SocketIO(app, async_mode="threading")
BOARD.setup()
parser = LoRaArgumentParser("A simple LoRa beacon")
parser.add_argument('--single', '-S', dest='single', default=False, action="store_true", help="Single transmission")
parser.add_argument('--wait', '-w', dest='wait', default=1, action="store", type=float, help="Waiting time between transmissions (default is 0s)")
thread = None
thread_lock = Lock()

class RobotLora(LoRa):
    def __init__(self):
        super(RobotLora, self).__init__(True)
        GPIO.setup([23, 24], GPIO.OUT)
        self.set_mode(MODE.STDBY)
        self.set_pa_config(pa_select=1)
        self.set_dio_mapping([0,0,0,0,0,0])
        self.set_freq(868)
        self.set_coding_rate(CODING_RATE.CR4_8)
        self.reset_ptr_rx()
        print("test")
        self.q = Queue()
        self.rx_reader = Thread(target=self.start_rx, args=(self.q,))
        #from lora.tx_command import LoRaBeacon

        # Set this variable to "threading", "eventlet" or "gevent" to test the
        # different async modes, or leave it set to None for the application to choose
        # the best option based on installed packages.

        #lora = LoRaBeacon(verbose=False)

    def tx_mode(self):
        GPIO.output(23, GPIO.HIGH)
        GPIO.output(24, GPIO.LOW)
        self.set_mode(MODE.TX)

    def rx_mode(self):
        GPIO.output(24, GPIO.HIGH)
        GPIO.output(23, GPIO.LOW)
        self.set_mode(MODE.RXCONT)

    def on_rx_done(self):
        print("\nRxDone")
        self.clear_irq_flags(RxDone=1)
        payload = self.read_payload(nocheck=True )
        self.set_mode(MODE.SLEEP)
        self.reset_ptr_rx()
        print(payload)
        info = payload #struct.unpack('<f', payload)
        #print(info)
        socketio.emit('update_info',{'data': info})
        self.rx_mode()
    
    def on_tx_done(self):
        print(self.get_irq_flags)

    def start_rx(self, q_in):
        #print("start rx")
        while(True):
            #print(self.get_irq_flags())
            if self.get_irq_flags()["rx_done"] == 1:
                self.on_rx_done()
            time.sleep(0.1)


lora = RobotLora()
args = parser.parse_args(lora)

@app.route('/')

def index():
    return render_template('index.html', async_mode=socketio.async_mode)


@socketio.event
def drive_event(message):
    print(lora.get_modem_status())
    #lora.start(message)
    command = [message] 
    lora.write_payload(command)
    lora.tx_mode()
    print(command)
    lora.rx_mode()

@socketio.event
def send_event(message):
    #print(type(message))
    package = list(struct.pack('>f', float(message[0])))
    package += list(struct.pack('>f', float(message[1])))
    lora.write_payload(package)
    lora.tx_mode()
    print(float(message[0]))
    print(float(message[1]))
    print(package)
    lora.rx_mode()

@socketio.event
def update_event(message):
    print(message)
    test =[20, 21, 22]
    #test = [ord(s.encode("ASCII", "ignore")) for s in message]
    print(test)
    lora.write_payload(test)
    lora.tx_mode()
    lora.rx_mode()

def background_thread():
    #Example of how to send server generated events to clients.
    count = 0
    while True:
        socketio.sleep(10)
        count += 1
        socketio.emit('my_response',
                    {'data': 'Server generated event', 'count': count})
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
    lora.set_coding_rate(CODING_RATE.CR4_8)
    lora.set_pa_config(pa_select=1)
    lora.set_dio_mapping([0,0,0,0,0,0])
    lora.set_preamble(10)
    lora.set_fifo_rx_base_addr(0)
    lora.clear_irq_flags(RxDone=1)
    lora.reset_ptr_rx()
    lora.rx_mode()
    assert(lora.get_agc_auto_on() == 1)
    #while(True):
        #print(lora.get_irq_flags())
        #time.sleep(0.5)
    lora.rx_reader.start()
    async_mode = None
    app.config['SECRET_KEY'] = 'secret!'
    socketio.run(app)
    lora.set_mode(MODE.STDBY)
    print(lora)
    

BOARD.teardown()