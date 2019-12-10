import cv2 as cv
import numpy as np

image = cv.imread('../test_images/sample_orange_line.JPG',0)
cv.imshow('Image',image)
cv.waitKey(0)
cv.destroyAllWindows()

'''
gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
cv.imshow('Gray',gray)
cv.waitKey(0)
cv.destroyAllWindows()
'''

edges = cv.Canny(image,50,50,apertureSize = 3)
cv.imshow('Edges', edges)
cv.waitKey(0)
cv.destroyAllWindows()

lines = cv.HoughLinesP(edges,1,np.pi/180,10)
for x1,y1,x2,y2 in lines[0]:
    cv.line(image,(x1,y1),(x2,y2),(0,255,0),2)
cv.imshow('Image',image)
cv.waitKey(0)
cv.destroyAllWindows()
