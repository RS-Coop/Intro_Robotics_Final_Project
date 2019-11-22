#!/usr/bin/env python
import rospy
import Drone_Control as dc

#Initializes a node for a drone
#rospy.init()

drone = dc.DroneController()

drone.run_node()
