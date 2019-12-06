#!/usr/bin/env python

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import cv2 as cv
import numpy as np
import argparse
from cv_bridge import CvBridge, CvBridgeError
import rospy
import intera_interface

def get_game_state(img_data, (edge_detection, window_name)):
    """The callback function to show image by using CvBridge and cv
       and converts image to the gamestate
    """
    bridge = CvBridge()
    try:
        img = bridge.imgmsg_to_cv2(img_data, "bgr8")
    except CvBridgeError, err:
        rospy.logerr(err)
        return

    #create a 2d array to hold the gamestate
    gamestate = [0,0,0,0,0,0,0,0,0]

    # Crop top of image
    img_crop = img[0:150, 0:900]


    # Define range of green color in grayscale
    lower_green = 40
    upper_green = 55

    # Threshold image to get only green colors
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

    # Origin adjustments
    x -= 3
    y -= 3
    w += 6
    h += 6

    # measure 1 cm in pixels
    cm = (w+h)/4

    buffer = cm/2

    tape_crop = img[y:y+h, x:x+w]

    # Crop frame for each board space
    top_left = img[y + 2*cm + buffer: y + 9*cm - buffer,  x + 2*cm + buffer: x + 9*cm - buffer]
    top_mid = img[y+2*cm + buffer: y + 9*cm,  x + 11*cm + buffer: x + 18*cm - buffer]
    top_right = img[y+2*cm + buffer: y + 10*cm - buffer,  x + 19*cm + buffer: x + 28*cm - buffer]
    mid_left = img[y + 11*cm + buffer: y + 18*cm - buffer,  x + 2*cm: x + 9*cm - buffer]
    mid_mid = img[y+11*cm + buffer: y + 18*cm - buffer,  x + 11*cm + buffer: x + 18*cm - buffer]
    mid_right = img[y+11*cm + 2*buffer: y + 18*cm,  x + 20*cm: x + 28*cm - buffer]
    bot_left = img[y + 20*cm + buffer: y + 29*cm - buffer,  x + 2*cm: x + 9*cm - buffer]
    bot_mid = img[y+20*cm + buffer: y + 29*cm - buffer,  x + 11*cm + buffer: x + 18*cm - buffer]
    bot_right = img[y+20*cm + 2*buffer: y + 29*cm - buffer,  x + 20*cm + buffer: x + 29*cm - buffer]

    board = [top_left, top_mid, top_right, mid_left, mid_mid, mid_right, bot_left, bot_mid, bot_right]
    kernel = np.ones((4,4),np.uint8)

    board_index = -1

    for space in board:

        board_index += 1

        # Find contours of edges
        edge = cv.Canny(space, 30, 100)

        # Dilate edges to fill in gaps
        dilation = cv.dilate(edge,kernel,iterations = 1)

        # Find contours in the board space
        im2, contours, hierarchy = cv.findContours(dilation, cv.RETR_EXTERNAL,cv.CHAIN_APPROX_NONE)

        # Find largest contour (the X or O)
        large_cnt_index = 0
        largest_peri = 0
        for i in range(len(contours)):
            perimeter = cv.arcLength(contours[i],True)
            if perimeter > largest_peri:
                largest_peri = perimeter
                large_cnt_index = i

        cv.drawContours(space, contours, large_cnt_index, (255,0,0), 3)
        largest_ct = contours[large_cnt_index]

        # Determine if the space is filled by an X or O
        if cv.contourArea(largest_ct) > 3500:
            # Calculate the solitity
            area = cv.contourArea(largest_ct)
            hull = cv.convexHull(largest_ct)
            hull_area = cv.contourArea(hull)
            try:
                solidity = float(area)/hull_area
            except ZeroDivisionError:
                solidity = -100

            # fill the gamestate with the right sign
            if(solidity > 0.5):
                    gamestate[board_index] = 2
            else:
                    gamestate[board_index] = 1

    #print the gamestate
    gamestate = np.reshape(gamestate, (3,3))
    print gamestate

    # resize final image
    #res = cv.resize(top_left, None,fx=0.25, fy=0.25, interpolation = cv.INTER_CUBIC)

    # display image
    cv.imshow('image1', top_left)
    cv.waitKey(0)
    cv.destroyAllWindows()

def main():
    """Camera Display

    Hand Camera Ranges
      - exposure: [0.01-100]
      - gain: [0-255]
    Head Camera Ranges:
      - exposure: [0-100], -1 for auto-exposure
      - gain: [0-79], -1 for auto-gain
    """
    rp = intera_interface.RobotParams()
    valid_cameras = rp.get_camera_names()
    if not valid_cameras:
        rp.log_message(("Cannot detect any camera_config"
          " parameters on this robot. Exiting."), "ERROR")
        return
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                description=main.__doc__)
    parser.add_argument(
        '-c', '--camera', type=str, default="head_camera",
        choices=valid_cameras, help='Setup Camera Name for Camera Display')
    parser.add_argument(
        '-r', '--raw', action='store_true',
        help='Specify use of the raw image (unrectified) topic')
    parser.add_argument(
        '-e', '--edge', action='store_true',
        help='Streaming the Canny edge detection image')
    parser.add_argument(
        '-g', '--gain', type=int,
        help='Set gain for camera (-1 = auto)')
    parser.add_argument(
        '-x', '--exposure', type=float,
        help='Set exposure for camera (-1 = auto)')
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node('camera_display', anonymous=True)
    cameras = intera_interface.Cameras()
    if not cameras.verify_camera_exists(args.camera):
        rospy.logerr("Could not detect the specified camera, exiting.")
        return
    rospy.loginfo("Opening camera '{0}'...".format(args.camera))
    cameras.start_streaming(args.camera)
    rectify_image = not args.raw
    use_canny_edge = args.edge
    cameras.set_callback(args.camera, get_game_state,
          rectify_image=rectify_image, callback_args=(use_canny_edge, args.camera))

    # optionally set gain and exposure parameters
    if args.gain is not None:
        if cameras.set_gain(args.camera, args.gain):
            rospy.loginfo("Gain set to: {0}".format(cameras.get_gain(args.camera)))

    if args.exposure is not None:
        if cameras.set_exposure(args.camera, args.exposure):
            rospy.loginfo("Exposure set to: {0}".format(cameras.get_exposure(args.camera)))

    def clean_shutdown():
        print("Shutting down camera_display node.")
        cv.destroyAllWindows()

    rospy.on_shutdown(clean_shutdown)
    rospy.loginfo("Camera_display node running. Ctrl-c to quit")
    rospy.spin()


if __name__=='__main__':
    main()
