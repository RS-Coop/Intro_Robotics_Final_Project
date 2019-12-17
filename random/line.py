import cv2 as cv
import numpy as np

O_TOP = 'outer top'
O_BOTTOM = 'outer bottom'
O_LEFT = 'outer left'
O_RIGHT = 'outer right'

#Inner
I_TOP = 'inner top'
I_BOTTOM = 'inner bottom'
I_LEFT = 'inner left'
I_RIGHT = 'inner right'

def get_band_averages(mask):
    band_avgs = {}
    height = mask.shape[0]
    width = mask.shape[1]
    print(height)
    print(width)

    sum_y = 0
    sum_x = 0
    num_pixels = 0

    for row in range(5):
        for col in range(width):
            if mask[row, col] == 255:
                sum_x += col
                sum_y += row
                num_pixels += 1
    print(num_pixels)
    if num_pixels > 5:
        band_avgs.update({O_TOP:(sum_x/num_pixels, sum_y/num_pixels)})
    else:
        band_avgs.update({O_TOP:(None,None)})
###############################################################################
    sum_y = 0
    sum_x = 0
    num_pixels = 0

    for row in range(height-6, height):
        for col in range(width):
            if mask[row, col] == 255:
                sum_x += col
                sum_y += row
                num_pixels += 1

    if num_pixels > 5:
        band_avgs.update({O_BOTTOM:(sum_x/num_pixels, sum_y/num_pixels)})
    else:
        band_avgs.update({O_BOTTOM:(None,None)})
###############################################################################
    sum_y = 0
    sum_x = 0
    num_pixels = 0

    for row in range(height):
        for col in range(5):
            if mask[row, col] == 255:
                sum_x += col
                sum_y += row
                num_pixels += 1

    if num_pixels > 5:
        band_avgs.update({O_LEFT:(sum_x/num_pixels, sum_y/num_pixels)})
    else:
        band_avgs.update({O_LEFT:(None,None)})
###############################################################################
    sum_y = 0
    sum_x = 0
    num_pixels = 0

    for row in range(height):
        for col in range(width-6, width):
            if mask[row, col] == 255:
                sum_x += col
                sum_y += row
                num_pixels += 1

    if num_pixels > 5:
        band_avgs.update({O_RIGHT:(sum_x/num_pixels, sum_y/num_pixels)})
    else:
        band_avgs.update({O_RIGHT:(None,None)})
###############################################################################
    sum_y = 0
    sum_x = 0
    num_pixels = 0

    for row in range(5):
        for col in range(width):
            if mask[row, col] == 255:
                sum_x += col
                sum_y += row
                num_pixels += 1

    if num_pixels > 5:
        band_avgs.update({I_TOP:(sum_x/num_pixels, sum_y/num_pixels)})
    else:
        band_avgs.update({I_TOP:(None,None)})
###############################################################################
    sum_y = 0
    sum_x = 0
    num_pixels = 0

    for row in range(height-6, height):
        for col in range(width):
            if mask[row, col] == 255:
                sum_x += col
                sum_y += row
                num_pixels += 1

    if num_pixels > 5:
        band_avgs.update({I_BOTTOM:(sum_x/num_pixels, sum_y/num_pixels)})
    else:
        band_avgs.update({I_BOTTOM:(None,None)})
###############################################################################
    sum_y = 0
    sum_x = 0
    num_pixels = 0

    for row in range(height):
        for col in range(5):
            if mask[row, col] == 255:
                sum_x += col
                sum_y += row
                num_pixels += 1

    if num_pixels > 5:
        band_avgs.update({I_LEFT:(sum_x/num_pixels, sum_y/num_pixels)})
    else:
        band_avgs.update({I_LEFT:(None,None)})
###############################################################################
    sum_y = 0
    sum_x = 0
    num_pixels = 0

    for row in range(height):
        for col in range(width-6, width):
            if mask[row, col] == 255:
                sum_x += col
                sum_y += row
                num_pixels += 1

    if num_pixels > 5:
        band_avgs.update({I_RIGHT:(sum_x/num_pixels, sum_y/num_pixels)})
    else:
        band_avgs.update({I_RIGHT:(None,None)})
###############################################################################
    return band_avgs

if __name__=='__main__':
    image = cv.imread('../test_images/orange_edge.jpg')
    cv.imshow('Image',image)
    cv.waitKey(0)
    cv.destroyAllWindows()

    #First filter for color
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    lower = np.array([5,100,150])
    upper = np.array([20,255,255])
    mask = cv.inRange(hsv, lower, upper)
    cv.imshow('Mask', mask)
    cv.waitKey(0)
    cv.destroyAllWindows()

    pos_avgs = get_band_averages(mask)
    print(pos_avgs)
