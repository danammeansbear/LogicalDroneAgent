# -*- coding: utf-8 -*-
"""
Created on Sun Jun 13 15:31:54 2021

@author: adam
"""
import requests
import json
import pyttsx3
engine = pyttsx3.init()
voices = engine.getProperty('voices')
engine.setProperty('voice', voices[0].id)


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
def hellodrone():
    # Connect to the Vehicle.
    print("Connecting to vehicle on: %s" % (connection_string,))
    vehicle = connect(connection_string, wait_ready=True)
    
    # Get some vehicle attributes (state)
    print ("Get some vehicle attribute values:")
    print (" GPS: %s" % vehicle.gps_0)
    print (" Battery: %s" % vehicle.battery)
    print (" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print (" Is Armable?: %s" % vehicle.is_armable)
    print (" System status: %s" % vehicle.system_status.state)
    print (" Mode: %s" % vehicle.mode.name)    # settable
    
    speak("Get some vehicle attribute values:")
    speak(" GPS: %s" % vehicle.gps_0)
    speak(" Battery: %s" % vehicle.battery)
    speak(" Last Heartbeat: %s" % vehicle.last_heartbeat)
    speak(" Is Armable?: %s" % vehicle.is_armable)
    speak(" System status: %s" % vehicle.system_status.state)
    speak(" Mode: %s" % vehicle.mode.name)    # settable
    
    # Close vehicle object before exiting script
    vehicle.close()
    
    # Shut down simulator
    sitl.stop()
    print("Completed")
    speak("completed")