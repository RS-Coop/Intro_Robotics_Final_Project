import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Image
from Intro_Robotics_Final_Project.msg import DroneCommand, TaggedImage, DroneMovementCommand
from Globals import Globals as G
import time

#This class deals with interactions between a single drone
'''
Overall: Working good, could eliminate camera joint sub, and should tweak
the intensity for drone movements. But note that intensity is limited to -1,1
'''
class DroneController:
    pre_land = Twist()
    pre_land.linear.z = -1 #Can maybe increase this

    post_takeoff = Twist()
    post_takeoff.linear.z = -0.1
    last_image_time = 0.0

    frame_count = 0

    # Amount to move forward
    linear_x_movement_amount = 0.5
    linear_y_movement_amount = 0.2
    angle_rotate_amount = 0.3

    #Initializes object with pubs and subs for speciic namespace
    def __init__(self, namespace='/bebop'):
        self.ID = namespace
        #Pubs
        self.pilot_pub = rospy.Publisher(namespace + '/cmd_vel', Twist, queue_size=1)
        self.takeoff_pub = rospy.Publisher(namespace + '/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher(namespace + '/land', Empty, queue_size=1)
        self.calibrate_pub = rospy.Publisher(namespace + '/flattrim', Empty, queue_size=1)#NOTE: Flat trim might have problems
        self.emergency_pub = rospy.Publisher(namespace + '/reset', Empty, queue_size=1)

        self.camera_joint_pub = rospy.Publisher(namespace + '/camera_control', Twist, queue_size=1)
        self.camera_image_pub = rospy.Publisher('/swarm/drone_image', TaggedImage, queue_size=1)

        #Subs
        self.pilot_sub = rospy.Subscriber('/swarm/drone_command', DroneMovementCommand, self.drone_command_callback)
        # self.camera_joint_sub = rospy.Subscriber(namespace + '/joint_states', JointState, self.camera_joint_callback)
        self.camera_image_sub = rospy.Subscriber(namespace + '/image_raw', Image, self.image_callback)

        rospy.sleep(2.0)

################################################################################
#Main methods and publisher methods

    #Loop to be run inside node script
    #TODO:
    #NOTE: Not sure if this is exactly what we need
    def run_node(self):
        while not rospy.is_shutdown():
            continue

    #If command is for this drone then move
    #DONE
    # def move_drone_callback(self, command):
    #     self.move_drone(command)
    #     print(self.ID,': command recieved->',command.cmd_type[0])

    #Move drone
    #DONE:
    #NOTE: Need to re-test this
    # def move_drone(self, command):
    #     #Command is movement type
    #     if command.cmd_type[0] == G.EMERGENCY:
    #         self.failsafe()

    #     elif command.cmd_type[0] == G.LAND:
    #         self.land()

    #     elif command.cmd_type[0] == G.TAKEOFF:
    #         self.takeoff()

    #     else:
            # movement = Twist()
    #         default_intensity = 0.2

    #         for i in range(len(command.cmd_type)):
    #             if command.intensity[i] != 0:
    #                 default_intensity = command.intensity[i]

    #             if command.cmd_type[i] == G.X:
    #                 movement.linear.x = default_intensity * command.direction[i]

    #             elif command.cmd_type[i] == G.Y:
    #                 movement.linear.y = default_intensity * command.direction[i]

    #             elif command.cmd_type[i] == G.Z:
    #                 movement.linear.z = default_intensity * command.direction[i]

    #             elif command.cmd_type[i] == G.THETA:
    #                 # +==CCW, -==CW, i.e. follows unit circle
    #                 movement.angular.z = default_intensity * command.direction[i]

    #         self.pilot_pub.publish(movement)

    #Tells drone to takeoff
    #DONE
    def takeoff(self):
        self.calibrate_pub.publish(Empty())
        self.move_camera()

        rospy.sleep(2.0)

        self.takeoff_pub.publish(Empty())

        rospy.sleep(2.0)

        # self.pilot_pub.publish(self.post_takeoff)

        print("Flying")

    #Tells drone to land
    #DONE: Lowers drone a little bit before landing
    def land(self):
        self.pilot_pub.publish(self.pre_land) #Not sure this is needed

        rospy.sleep(1.0)

        self.land_pub.publish(Empty())

    #Set the new camera joint states based on joint_data
    #DONE: Defualt behavior is to move camera straight down
    #NOTE: API is unstable as per documentation, joint_data is Twist
    def move_camera(self, joint_data=None):
        if joint_data == None:
            joint_data = Twist()
            joint_data.angular.y = -90.0
            joint_data.angular.z = 0.0
            self.camera_joint_pub.publish(joint_data)

    #Stop moving the drone and land
    #DONE: This works well
    #NOTE: Using this in ros shutdown hooks works well
    def failsafe(self):
        self.emergency_pub.publish(Empty())
        self.land_pub.publish(Empty())

    def run_movement_command(self, movementCommand):
        if movementCommand == G.GO_FORWARD:
            self.fly_drone_forward()
        elif movementCommand == G.GO_BACKWARD:
            self.fly_drone_backward()
        elif movementCommand == G.GO_RIGHT:
            self.fly_drone_right()
        elif movementCommand == G.GO_LEFT:
            self.fly_drone_left()
        elif movementCommand == G.TURN_RIGHT:
            self.rotate_drone_right()
        elif movementCommand == G.TURN_LEFT:
            self.rotate_drone_left()
        elif movementCommand == G.DO_EMERGENCY:
            self.failsafe()
        elif movementCommand == G.DO_LAND:
            self.land()
        elif movementCommand == G.DO_TAKEOFF:
            self.takeoff()

    def fly_drone_forward(self):
        movement = Twist()
        movement.linear.x = self.linear_x_movement_amount
        self.pilot_pub.publish(movement)

    def fly_drone_backward(self):
        movement = Twist()
        movement.linear.x = -self.linear_x_movement_amount
        self.pilot_pub.publish(movement)

    def fly_drone_right(self):
        movement = Twist()
        movement.linear.y = -self.linear_y_movement_amount
        self.pilot_pub.publish(movement)

    def fly_drone_left(self):
        movement = Twist()
        movement.linear.y = self.linear_y_movement_amount
        self.pilot_pub.publish(movement)

    def rotate_drone_right(self):
        movement = Twist()
        movement.angular.z = self.angle_rotate_amount
        self.pilot_pub.publish(movement)

    def rotate_drone_left(self):
        movement = Twist()
        movement.angular.z = -self.angle_rotate_amount
        self.pilot_pub.publish(movement)

################################################################################
#Callbacks
    '''
    #Callback to update camera joint data
    #DONE:
    #NOTE: Probably dont need this
    def camera_joint_callback(self, data):
        self.curr_cam_joint = data
    '''

    ##### PUBLISH CALLBACKS

    #Callback to publish image data
    #DONE: Gather image data, tag it and then publish it
    #NOTE: Havent tested this yet
    def image_callback(self, data):
        if time.time()-self.last_image_time > 1.5:
            print('Publishing Image')
            drone_image = TaggedImage()
            drone_image.image = data
            drone_image.drone_id = self.ID

            self.camera_image_pub.publish(drone_image)
            self.last_image_time = time.time()

    ##### SUBSCRIBE CALLBACKS
    def drone_command_callback(self, data):
        movementCommand = data.movement_command
        print("Recieved Command: ", movementCommand)

        self.run_movement_command(movementCommand)

    # def image_callback(self, data):
    #     self.frame_count += 1
    #     if self.frame_count > 100:
    #         print('Publishing Image')
    #         drone_image = TaggedImage()
    #         drone_image.image = data
    #         drone_image.drone_id = self.ID
    #
    #         self.camera_image_pub.publish(drone_image)
    #         self.frame_count = 0

################################################################################
#Tests
    #PASS
    #NOTE: Landing == Falls out of sky
    def test_takeoffandland(self):
        self.takeoff()
        rospy.sleep(5.0)
        self.land()

    #PASS
    #NOTE: Intensity for movements probably needs to be increased
    def test_failsafe(self):
        self.takeoff()
        rospy.sleep(2.0)
        command = DroneCommand()
        command.cmd_type.append(G.Z)
        command.drone_id = self.ID
        command.intensity.append(0)
        command.direction.append(-1)

        self.move_drone(command)
        print('Waiting for SIGCHLD, will terminate in 10 seconds')
        rospy.sleep(10.0)
        self.failsafe()

    #PASS
    #NOTE: Intensity for movements probably needs to be increased
    def test_moveforward(self):
        self.takeoff()
        rospy.sleep(2.0)
        command = DroneCommand()
        command.cmd_type.append(G.X)
        command.drone_id = self.ID
        command.intensity.append(0)
        command.direction.append(1)

        self.move_drone(command)
        rospy.sleep(5.0)
        self.land()

    #Not yet tested
    #Mostly just need to tet camera data
    def test_generalfunction():
        self.takeoff()
        rospy.sleep(2.0)
        self.move_camera()

        command = DroneCommand()
        command.cmd_type.append(G.X)
        command.drone_id = self.ID
        command.intensity.append(0.3)
        command.direction.append(1)

        self.move_drone(command)
        rospy.sleep(10.0)

        command.cmd_type.append(G.THETA)
        command.drone_id = self.ID
        command.intensity.append(0)
        command.direction.append(1)

        self.move_drone(command)
        rospy.sleep(1.0)

        self.land
