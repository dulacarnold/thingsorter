import zmq
import random
import sys
import time
import json

port = "5556"
if len(sys.argv) > 1:
    port =  sys.argv[1]
    int(port)

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://localhost:5556")

monitor_socket = context.socket(zmq.PULL)
monitor_socket.bind("tcp://localhost:5559")

while True:
    time.sleep(1)
    msg = json.dumps({'timestamp': '{}'.format(time.time()),
                      'control': 'RUN'})
    print(msg)
    mon_msg = monitor_socket.recv_json()
    print(mon_msg)
    socket.send_string(msg)
