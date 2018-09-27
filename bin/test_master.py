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
socket.bind("tcp://*:%s" % port)

while True:
    time.sleep(1)
    msg = json.dumps({'timestamp': '{}'.format(time.time()),
                      'control': 'RUN'})
    print(msg)
    socket.send_string(msg)
