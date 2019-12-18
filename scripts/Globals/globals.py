import numpy as np

class Globals:
    # State definitions
    TAKEOFF = 0
    CENTER_QR = 1
    DETERMINE_NEXT_LINE = 2
    NAVIGATE_TO_LINE = 6
    FOLLOW_LINE = 4
    MOVE_ONTO_LINE = 3
    LAND = 5
    KILL = 99

    # Error for QR BB
    QR_ERROR = 10

    # Angle bound to determine if theta needs correcting
    ANGLE_BOUND = 15

    # Line Colors
    ORANGE = "orange"
    PURPLE = "purple"
    BLUE = "blue"
    WHITE = "white"

    # Color ranges
    LINE_COLORS = {
        ORANGE:[np.array([5,100,150]), np.array([20,255,255])],
        PURPLE:[np.array([275,100,150]), np.array([285,255,255])],
        BLUE:[np.array([100,50,50]), np.array([115,255,255])],
        WHITE:[np.array([0,0,235]), np.array([255,20,255])]
    }

    # Center of bebop center
    BEBOP_CENTER = (320, 184)


    DEFAULT_ANGLE_ERROR = 10

    # Max photos center
    MAX_CENTER = (1512, 2016)
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
    EMERGENCY = 'emergency'
    TAKEOFF = 'takeoff'
    LAND = 'land'
    X = 'x'
    Y = 'y'
    Z = 'z'
    THETA = 'theta'
