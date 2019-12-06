#!/usr/bin/env python
import numpy as np
import cv2 as cv


# Load a color image
img = cv.imread('x5_g15.png', cv.IMREAD_GRAYSCALE)
img_crop = img[0:150, 0:900]


# Define range of green color in HSV
lower_green = 40
upper_green = 55

# Threshold HSV image to get only green colors
mask = cv.inRange(img_crop, lower_green, upper_green)

# Find contours of edges
edged = cv.Canny(mask, 30, 200)

im, contours, hierarchy = cv.findContours(edged, cv.RETR_EXTERNAL,cv.CHAIN_APPROX_NONE)

# Find largest contour (the green tape)
large_cnt_index = 0
largest_peri = 0
for i in range(len(contours)):
    perimeter = cv.arcLength(contours[i],True)
    if perimeter > largest_peri:
        largest_peri = perimeter
        large_cnt_index = i

# get coordinates of tile
x,y,w,h = cv.boundingRect(contours[large_cnt_index])

cv.drawContours(img, contours, large_cnt_index, (255,0,0), 2)

# display image
cv.imshow('image1', img)
cv.waitKey(0)
cv.destroyAllWindows()
