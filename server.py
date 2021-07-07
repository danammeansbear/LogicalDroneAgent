#! /usr/local/bin/python3
import zmq
import time
import random
from dronemission import mission
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:12345")

while True:
	message = str(mission.mqlat) + " " + str(mission.mqlong) + " " + str(mission.mqalt)
	socket.send_string(message)
	print(message)
	time.sleep(1)
