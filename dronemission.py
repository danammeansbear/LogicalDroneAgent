
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil

# -*- coding: utf-8 -*-
"""
Created on Jun 25 15:31:54 2021

@author: adam
"""
#! /usr/local/bin/python3
import zmq
import time
import requests
import json
import pyttsx3
engine = pyttsx3.init()
voices = engine.getProperty('voices')
engine.setProperty('voice', voices[0].id)
context = zmq.Context()
socket = context.socket(zmq.PUB)
try:
    socket.bind("tcp://*:12346")
except zmq.error.ZMQError:
    print ("socket already in use, restarting")
    socket.close()
    context.destroy()
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:12346")

def speak(audio):
    engine.say(audio)
    engine.runAndWait()


print ("Start simulator (SITL)")
speak("Start simulator (SITL)")
import dronekit_sitl
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

# Import DroneKit-Python
from dronekit import connect, VehicleMode

def mission():
    #Set up option parsing to get connection string
    import argparse  
    parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
    parser.add_argument('--connect', 
                       help="vehicle connection target string. If not specified, SITL automatically started and used.")
    args = parser.parse_args()
    
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
    
    

    

        #return zeroMqLocation(mqalt, mqlat, mqlong)
    def get_location_metres(original_location, dNorth, dEast):
        """
        Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
        specified `original_location`. The returned Location has the same `alt` value
        as `original_location`.
        The function is useful when you want to move the vehicle around specifying locations relative to 
        the current vehicle position.
        The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
        For more information see:
        http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        """
        earth_radius=6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))
    
        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        return LocationGlobal(newlat, newlon,original_location.alt)
    
    
    def get_distance_metres(aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.
        This method is an approximation, and will not be accurate over large distances and close to the 
        earth's poles. It comes from the ArduPilot test code: 
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    
    
    
    def distance_to_current_waypoint():
        """
        Gets distance in metres to the current waypoint. 
        It returns None for the first waypoint (Home location).
        """
        nextwaypoint = vehicle.commands.next
        if nextwaypoint==0:
            return None
        missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
        lat = missionitem.x
        lon = missionitem.y
        alt = missionitem.z
        targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
        distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
        return distancetopoint
    
    
    def download_mission():
        """
        Download the current mission from the vehicle.
        """
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready() # wait until download is complete.
    
    
    
    def adds_square_mission(aLocation, aSize):
        """
        Adds a takeoff command and four waypoint commands to the current mission. 
        The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).
        The function assumes vehicle.commands matches the vehicle mission state 
        (you must have called download at least once in the session and after clearing the mission)
        """	
    
        cmds = vehicle.commands
    
        print(" Clear any existing commands")
        cmds.clear() 
        
        print(" Define/add new commands.")
        # Add new commands. The meaning/order of the parameters is documented in the Command class. 
         
        #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
    
        #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
        point1 = get_location_metres(aLocation, aSize, -aSize)
        point2 = get_location_metres(aLocation, aSize, aSize)
        point3 = get_location_metres(aLocation, -aSize, aSize)
        point4 = get_location_metres(aLocation, -aSize, -aSize)
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))
        #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))    
    
        print(" Upload new commands to vehicle")
        cmds.upload()
    
    
    def arm_and_takeoff(aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """
    
        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)
    
            
        print("Arming motors")
        # Copter should arm in GUIDED mode
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
    
        while not vehicle.armed:      
            print(" Waiting for arming...")
            time.sleep(1)
    
        print("Taking off!")
        vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
    
        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
            if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
                print("Reached target altitude")
                break
            time.sleep(1)
    
            
    print('Create a new mission (for current location)')
    adds_square_mission(vehicle.location.global_frame,50)
    
    
    # From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
    arm_and_takeoff(10)
    
    print("Starting mission")
    # Reset mission set to first (0) waypoint
    vehicle.commands.next=0
    
    # Set mode to AUTO to start mission
    vehicle.mode = VehicleMode("AUTO")
    
    
    # Monitor mission. 
    # Demonstrates getting and setting the command number 
    # Uses distance_to_current_waypoint(), a convenience function for finding the 
    #   distance to the next waypoint.
    #Distance Calculator 

    #fuck I hope lol
    #would like to use gimble angle to try and get the distance...
    #consider sidea as the Altitude of the drone
    sidea = float(vehicle.location.global_relative_frame.alt)
    #sideb is the known distance if at all
    #sideb = float(input("Gimble angle : "))
    #sidec is basically the ray trace distnace from drone to object


    lat = int(vehicle.location.global_relative_frame.lat);
    long = int(vehicle.location.global_relative_frame.lon);
    alt = int(vehicle.location.global_relative_frame.alt);

    #not the best way to do this
    #calculate the cos of distance of drone to the object
    #given the drones altitude or sidea
    cameraObjectDistance = sidea // (math.cos(sidea));
    #calculate distance from object
    gimbleAngle = 60; 
    objectDistance = math.sqrt((pow(cameraObjectDistance,2)) - (pow(sidea,2)));
    distanceTrue = (alt * math.sin(gimbleAngle))/(math.sin(alt));
    #vehicle.location.global_relative_frame.alt) + " " + str(vehicle.location.global_relative_frame.lat) + " " + str(vehicle.location.global_relative_frame.lon
    targetLocation = (lat + objectDistance,long,alt - alt + 2);
    print("Drone ray trace distance: " + str(cameraObjectDistance));
    print("object Distance: " + str(objectDistance));
    print("target location:" + str(targetLocation));
    print("distance True :" + str(distanceTrue));

    while True:
        nextwaypoint=vehicle.commands.next
        print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
        #message = str(vehicle.location.global_relative_frame.alt) + " " + str(vehicle.location.global_relative_frame.lat) + " " + str(vehicle.location.global_relative_frame.lon + " " + str(x + b) + " "+ str(y) + " " + str(z - z + 2))
        #socket.send_string(message)
        LockedTargetLocation = str(lat + distanceTrue) + " " + str(long) + " " + str(alt - alt + 2)
        message = str(vehicle.location.global_relative_frame.alt) + " " + str(vehicle.location.global_relative_frame.lat) + " " + str(vehicle.location.global_relative_frame.lon) + " " + LockedTargetLocation
        print(message)
        print("Drone ray trace distance: " + str(cameraObjectDistance))
        print("Target Distance: " + str(objectDistance))
        print(targetLocation)
        #LockedTargetLocation = str(x + b) + " " + str(y) + " " + str(z - z + 2)
        socket.send_string(LockedTargetLocation)
        socket.send_string(message)
        socket.send_string(objectDistance)
        socket.send_string(targetLocation)
        socket.send_string(distanceTrue)
        print(LockedTargetLocation)
        time.sleep(1)
        if nextwaypoint==3: #Skip to next waypoint
            print('Skipping to Waypoint 5 when reach waypoint 3')
            vehicle.commands.next = 5
        if nextwaypoint==5: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
            print("Exit 'standard' mission when start heading to final waypoint (5)")
            break;
        time.sleep(1)
    
    print('Return to launch')
    vehicle.mode = VehicleMode("RTL")
    
    
    #Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()
    socket.close()
    
    # Shut down simulator if it was started.
    if sitl is not None:
        sitl.stop()