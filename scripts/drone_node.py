#!/usr/bin/env python
import rospy
from Drone_Control import DroneController

#Initializes a node for a drone
#NOTE: Might need to change this using cmd arguments for multiple drones
rospy.init_node('Drone1')

#Initializes a drone
drone = DroneController()

#Handler for rospy shutdown
rospy.on_shutdown(drone.land)

print("Ready to fly.")

#Runs loop
drone.run_node()

print("Landed")
