import rospy
from nav_msgs.msg import Odometry

#This class deals with
class SwarmController:
    'drones = [] #List of DroneController objects'#Maybe not this

    def __init__(self):
        #Initialize pubs and subs
        'drone_command_pub = rospy.Publisher'

        #NOTE: Theses msg types and topics might not be accurate
        drone_loc_sub = rospy.Subscriber('multi_agent/current_locations', Odom_Tag, queue_size=1)
        location_req_sub = rospy.Subscriber('multi_agent/location_requests', Odometry, queue_size=1)

    def dispatch_drone():
        pass
