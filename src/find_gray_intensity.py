#!/usr/bin/env python
import numpy as np
import cv2 as cv


# Load a color image
img = cv.imread('gray_tape.png', cv.IMREAD_GRAYSCALE)

avg = int(np.average(img))
median = int(np.median(img))

print avg, median

# Define range of green color in HSV
lower_green = avg - 10
upper_green = avg + 10

# Threshold HSV image to get only green colors
mask = cv.inRange(img, lower_green, upper_green)

# display image
cv.imshow('image1', img)
cv.waitKey(0)
cv.destroyAllWindows()
