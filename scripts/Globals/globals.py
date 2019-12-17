import numpy as np

class Globals:
    # State definitions
    TAKEOFF = -1
    CENTER_QR = 0
    DETERMINE_NEXT_LINE = 1
    NAVIGATE_TO_LINE = 2
    FOLLOW_LINE = 3
    MOVE_ONTO_LINE = 4
    LAND = 5

    # Error for QR BB
    QR_ERROR = 10

    # Angle bound to determine if theta needs correcting
    ANGLE_BOUND = 15

    # Line Colors
    ORANGE = "orange"
    PURPLE = "purple"
    BLUE = "blue"

    # Color ranges
    LINE_COLORS = {
        ORANGE:[np.array([5,100,150]), np.array([15,255,255])],
        PURPLE:[np.array([275,100,150]), np.array([285,255,255])],
        BLUE:[np.array([100,50,50]), np.array([115,255,255])]
    }

    # Center of bebop center
    BEBOP_CENTER = (320, 184)

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
