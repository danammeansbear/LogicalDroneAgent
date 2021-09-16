# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import math 
#! /usr/local/bin/python3
import zmq
import time
import random
from dronemission import mission
from dronekit import connect, VehicleMode
#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
context = zmq.Context()
socket = context.socket(zmq.PUB)
try:
    socket.connect("tcp://localhost:12345")
except zmq.error.ZMQError:
    print ("socket already in use, restarting")
    #sys.exit()
    socket.close()
    context.destroy()
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.connect("tcp://localhost:12345")

connection_string = args.connect
sitl = None


#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
   
    


#Distance Calculator 

#fuck I hope lol
#would like to use gimble angle to try and get the distance...
#consider sidea as the Altitude of the drone
sidea = float(vehicle.location.global_relative_frame.alt)
#sideb is the known distance if at all
#sideb = float(input("Gimble angle : "))
#sidec is basically the ray trace distnace from drone to object

#not the best way to do this
#calculate the cos of distance of drone to the object
#given the drones altitude or sidea
c = sidea // (math.cos(sidea));
#calculate b
b = math.sqrt((pow(c,2)) - (pow(sidea,2)));

lat = int(78.81);
long = int(77.33);
alt = int(vehicle.location.global_relative_frame.alt);

#vehicle.location.global_relative_frame.alt) + " " + str(vehicle.location.global_relative_frame.lat) + " " + str(vehicle.location.global_relative_frame.lon

z = lat;
x = long;
y = alt;
targetLocation = (x + b,y,z - z + 2);
print("Drone ray trace distance: " + str(c));
print("Target Distance: " + str(b));
print(targetLocation);
while True:
	LockedTargetLocation = str(x + b) + " " + str(y) + " " + str(z - z + 2)
	socket.send_string(LockedTargetLocation)
	print(LockedTargetLocation)
	time.sleep(1)
socket.close()