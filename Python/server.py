#! /usr/local/bin/python3
import zmq
import time
import random
from dronemission import ZeroMqTest
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:12345")

while True:
	message = str(ZeroMqTest.mqlat) + " " + str(ZeroMqTest.mqlong) + " " + str(ZeroMqTest.mqalt)
	socket.send_string(message)
	print(message)
	time.sleep(1)
