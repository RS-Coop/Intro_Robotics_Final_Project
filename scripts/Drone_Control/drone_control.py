import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Image
#This class deals with interactions between a single drone
class DroneController:
    self.curr_odom = None
    self.curr_cam_joint = None

    #Initializes object with pubs and subs for speciic namespace
    def __init__(self, namespace):
        self.ID = namespace
        #Initiate pubs and subs for single drone
        self.takeoff = rospy.Publisher(namespace + '/takeoff', Empty, queue_size=1)
        self.land = rospy.Publisher(namespace + '/land', Empty, queue_size=1)
        #NOTE: Flat trim might have problems
        self.calibrate = rospy.Publisher(namespace + '/flattrim', Empty, queue_size=1)

        self.pilot_pub = rospy.Publisher(namespace + '/cmd_vel', Twist, queue_size=1)
        self.camera_joint_pub = rospy.Publisher(namespace + '/camera_control', Twist, queue_size=1)
        self.camera_image_pub = rospy.Publisher(namespace + '/drone_image', Image, queue_size=1)

        #NOTE:Need to figure out flow of commands from SwarmController
        'self.pilot_sub = rospy.Subscriber(namespace + )'
        self.odom_sub = rospy.Subscriber(namespace + '/odom', Odometry, sef.odom_callback)
        self.camera_joint_sub = rospy.Subscriber(namespace + '/joint_states', JointState, self.camera_joint_callback)
        self.camera_image_sub = rospy.Subscriber(namespace + '/image_raw', Image, self.image_callback)

        sleep(1.0)

    #Loop to be run inside node script
    #TODO
    def run_node(self):
        pass

    #Callback to update odometry
    #DONE
    def odom_callback(self, data):
        self.curr_odom = data

    #Callback to update camera data
    #DONE
    def camera_joint_callback(self, data):
        self.curr_cam_joint = data

    #Callback to update image data
    #DONE: Gather image data, tag it and then publish it
    #NOTE: Still need to create custom image, and make sure this works
    def image_callback(self, data):
        self.camera_image_pub.publish(data)

    #TODO: Calculate IK to move to location specified by odometry
    def move_drone(self, goal_loc):
        #goal_loc is place you want to be.
        pass

    #Tells drone to takeoff
    #DONE
    def takeoff(self):
        self.calibrate.publish(Empty())

        sleep(1.0)

        self.takeoff.publish(Empty())

    #Tells drone to land
    #DONE
    def land(self):
        self.land.publish(Empty())

    #Set the new camera joint states based on joint_data
    #DONE
    #NOTE: API is unstable as per documentation, joint_data is Twist
    def move_camera(self, joint_data):
        self.camera_joint_pub.publish(joint_data)

    #Stop moving the drone and land
    #TODO
    def failsafe(self):
        #Stop moving drone and land
        pass
