import cv2 as cv
import numpy as np

image = cv.imread('../test_images/sample_orange_line.JPG')
cv.imshow('Image',image)
cv.waitKey(0)
cv.destroyAllWindows()

#First filter for color
hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
lower_orange = np.array([5,100,150])
upper_orange = np.array([15,255,255])
mask = cv.inRange(hsv, lower_orange, upper_orange)
cv.imshow('Mask', mask)

cv.waitKey(0)
cv.destroyAllWindows()
result = cv.bitwise_and(image, image, mask=mask)
cv.imshow('Blob', result)
cv.waitKey(0)
cv.destroyAllWindows()

'''
gray = cv.cvtColor(result, cv.COLOR_HSV2GRAY)
cv.imshow('Gray',gray)
cv.waitKey(0)
cv.destroyAllWindows()
'''
_,_,gray = cv.split(result)


edges = cv.Canny(mask,50,150,apertureSize = 3)
cv.imshow('Edges', edges)
cv.waitKey(0)
cv.destroyAllWindows()

# lines = cv.HoughLinesP(edges,1,np.pi/180,10)
# for x1,y1,x2,y2 in lines[0]:
#     len = np.sqrt((x1-x2)**2+(y1-y2)**2)
#     print(len)
#     if len > 100:
#         cv.line(image,(x1,y1),(x2,y2),(0,255,0),2)
dims = image.shape
blank = np.zeros((dims[0],dims[1],3),np.uint8)
lines = cv.HoughLinesP(edges,1,np.pi/180,10,10)
angle = 0
num_lines = 0
sum_adj = 0
sum_opp = 0

for line in lines:
    for x1,y1,x2,y2 in line:
        len = np.sqrt((x1-x2)**2+(y1-y2)**2)
        if len > 5:
            cv.line(blank,(x1,y1),(x2,y2),(0,255,0),2)
            # print(line)
            adj = np.abs(y1-y2)
            opp = np.abs(x1-x2)
            if adj > 0:
                angle += np.arctan(opp/adj)
                print('Opposite: %d, Adjacent: %d', opp, adj)
                num_lines += 1
                sum_opp += opp
                sum_adj += adj

print('Number of lines: %d', num_lines)
print(sum_opp/num_lines, sum_adj/num_lines)
print("Angle w/t vertical: %d", np.degrees(angle/num_lines))
num = (float(sum_opp/num_lines))/float((sum_adj/num_lines))
print('num: ',num)
angle = np.arctan(num)
print("Angle w/t vertical: %d", np.degrees(angle), angle)

cv.imshow('Image',blank)
cv.waitKey(0)
cv.destroyAllWindows()

# lines = cv.HoughLines(edges,1,np.pi/180,10)
# sum = 0
# num = 0
# for line in lines:
#     for _,theta in line:
#         sum += theta
#         num += 1
#
# print(np.degrees(sum/num))