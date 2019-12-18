import numpy as np

class Globals:
    # State definitions
    TAKEOFF = 0
    SEARCH_QR = 1
    CENTER_QR = 2
    DETERMINE_NEXT_LINE = 3
    MOVE_ONTO_LINE = 4
    FOLLOW_LINE = 5
    LAND = 6
    NAVIGATE_TO_LINE = 7
    KILL = 99



    # Line Colors
    ORANGE = "orange"
    PURPLE = "purple"
    BLUE = "blue"

    # Color ranges
    LINE_COLORS = {
        ORANGE:[np.array([5,100,150]), np.array([20,255,255])],
        PURPLE:[np.array([275,100,150]), np.array([285,255,255])],
        BLUE:[np.array([100,50,50]), np.array([115,255,255])],
    }

    # Center of bebop center
    BEBOP_CENTER = (240, 428)
    # Error for QR BB
    QR_ERROR_X = 100
    QR_ERROR_Y = 50

    # Angle bound to determine if theta needs correcting
    ANGLE_BOUND = 25

    DEFAULT_ANGLE_ERROR = 10

    # Max photos center
    MAX_CENTER = (2016, 1512)
    MAX_X_ERROR = 400
    MAX_Y_ERROR = 500

    #Mask bands
    #Outer
    O_TOP = 'outer top'
    O_BOTTOM = 'outer bottom'
    O_LEFT = 'outer left'
    O_RIGHT = 'outer right'

    #Inner
    I_TOP = 'inner top'
    I_BOTTOM = 'inner bottom'
    I_LEFT = 'inner left'
    I_RIGHT = 'inner right'

    #Band width in pixels
    BAND_SIZE = 5
    #THRESHOLD
    BAND_THRESHOLD = 5

    #Drone commands
    # EMERGENCY = 'emergency'
    # TAKEOFF = 'takeoff'
    # LAND = 'land'
    # X = 'x'
    # Y = 'y'
    # Z = 'z'
    # THETA = 'theta'

    # DRONE MOVEMENTS
     
    DO_LAND = 0
    GO_FORWARD = 1
    GO_BACKWARD = 2
    GO_RIGHT = 3
    GO_LEFT = 4
    TURN_RIGHT = 5
    TURN_LEFT = 6
    DO_EMERGENCY = 7
    DO_TAKEOFF = 8