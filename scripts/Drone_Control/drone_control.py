import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Image
from Intro_Robotics_Final_Project.msg import DroneCommand, TaggedImage
#This class deals with interactions between a single drone
class DroneController:
    curr_odom = None
    curr_cam_joint = None

    #Initializes object with pubs and subs for speciic namespace
    def __init__(self, namespace='/bebop'):
        self.ID = namespace
        #Initiate pubs and subs for single drone
        self.takeoff = rospy.Publisher(namespace + '/takeoff', Empty, queue_size=1)
        self.land = rospy.Publisher(namespace + '/land', Empty, queue_size=1)
        self.calibrate = rospy.Publisher(namespace + '/flattrim', Empty, queue_size=1)#NOTE: Flat trim might have problems
        self.emergency = rospy.Publisher(namespace + '/reset', Empty, queue_size=1)

        self.pilot_pub = rospy.Publisher(namespace + '/cmd_vel', Twist, queue_size=1)
        self.camera_joint_pub = rospy.Publisher(namespace + '/camera_control', Twist, queue_size=1)
        self.camera_image_pub = rospy.Publisher('/swarm/drone_image', TaggedImage, queue_size=1)

        #NOTE:Need to figure out flow of commands from SwarmController
        self.pilot_sub = rospy.Subscriber('/swarm/drone_command', DroneCommand, self.move_drone_callback)
        self.camera_joint_sub = rospy.Subscriber(namespace + '/joint_states', JointState, self.camera_joint_callback)
        self.camera_image_sub = rospy.Subscriber(namespace + '/image_raw', Image, self.image_callback)

        rospy.sleep(1.0)

################################################################################
#Main methods and publisher methods

    #Loop to be run inside node script
    #DONE
    #NOTE: Not sure if this is exactly what we need
    def run_node(self):
        while not rospy.is_shutdown():
            continue

    #If command is for this drone then move
    #DONE
    def move_drone_callback(self, command):
        if command.drone_id == self.ID:
            self.move_drone(command)

    #Move drone
    #DONE
    def move_drone(self, command):
        #Command is movement type
        movement = Twist()
        intensity = 0.2

        if command.intensity != 0:
            default_intensity = command.intensity

        if command.cmd_type == 'x':
            movement.linear.x = intensity * command.direction

        elif command.cmd_type == 'y':
            movement.linear.y = intensity * command.direction

        elif command.cmd_type == 'z':
            movement.linear.z = intensity * command.direction

        elif command.cmd_type == 'angular':
            movement.angular.z = intensity * command.direction

        self.pilot_pub.publish(movement)

    #Tells drone to takeoff
    #DONE
    def takeoff(self):
        self.calibrate.publish(Empty())

        sleep(2.0)

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
    #DONE
    #NOTE: Not exactly sure what this does
    def failsafe(self):
        self.emergency.publish(Empty())
        self.land.publish(Empty())

################################################################################
#Callbacks

    #Callback to update camera joint data
    #DONE
    def camera_joint_callback(self, data):
        self.curr_cam_joint = data

    #Callback to publish image data
    #DONE: Gather image data, tag it and then publish it
    def image_callback(self, data):
        drone_image = TaggedImage()
        drone_image.image = data
        drone_image.drone_id = self.ID

        self.camera_image_pub.publish(drone_image)

################################################################################
#Tests

    #Test Methods
    def test_takeoffandland(self):
        self.takeoff
        sleep(5.0)
        self.land

    def test_moveforward(self):
        self.takeoff
        sleep(2.0)
        command = DroneCommand()
        command.cmd_type = 'x'
        command.intensity = 0
        commmand.direction = 1

        self.move_drone(command)
