# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
import math 
#Distance Calculator 

#fuck I hope lol

#consider sidea as the Altitude of the drone
sidea = int(input("drone altitude: "))
#sideb is the known distance if at all
sideb = int(input("Gimble angle : "))
#sidec is basically the ray trace distnace from drone to object

#not the best way to do this
#calculate the cos of distance of drone to the object
#given the drones altitude or sidea
c = int(sidea % math.cos(sidea));
#calculate b
b = math.sqrt(c**c) - (sidea**sidea);

print("Drone ray trace distance: " + str(c));
print("Target Distance: " + str(b));