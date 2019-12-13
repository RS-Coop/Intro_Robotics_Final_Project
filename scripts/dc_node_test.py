#!/usr/bin/env python
import rospy
from Drone_Control import DroneController

#Use sys.argv[1] for test type

#Initializes a node for a drone
#NOTE: Might need to change this using cmd arguments for multiple drones
rospy.init_node('Drone1')

#Initializes a drone
drone = DroneController()

#Handler for rospy shutdown
rospy.on_shutdown(drone.failsafe)
print("Ready to fly")

test_type = rospy.get_param("/Drone1/test_type", None)
#Runs test

if test_type=='no_fly' or test_type==None:
    print('No action')

elif test_type=='failsafe':
    drone.test_failsafe()

elif test_type=='takeoffandland':
    drone.test_takeoffandland()

elif test_type=='moveforward':
    drone.test_moveforward()

elif test_type=='generalfunction':
    drone.test_generalfunction()

print("Landed")
