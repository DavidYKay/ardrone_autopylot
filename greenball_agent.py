'''
greenball_agent.py - Python green-ball-tracking agent for AR.Drone autopilot program.

    To use this program with the AR.Drone Autopylot package, you should rename
    or symbolic-link the program as agent.py.

    Copyright (C) 2010 Simon D. Levy

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as 
    published by the Free Software Foundation, either version 3 of the 
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License 
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 You should also have received a copy of the Parrot Parrot AR.Drone 
 Development License and Parrot AR.Drone copyright notice and disclaimer 
 and If not, see 
   <https://projects.ardrone.org/attachments/277/ParrotLicense.txt> 
 and
   <https://projects.ardrone.org/attachments/278/ParrotCopyrightAndDisclaimer.txt>.

 Author gratefully acknowledges the help of Professor Joshua Stough in writing
 this program.
'''

from agent import *
import cv

# For OpenCV image display
WINDOW_NAME = 'Autopilot' 

# Object-detection threshold.  You may have to adjust this (between 0 and 255) for 
# for different lighting conditions.
THRESHOLD = 100

# PID parameters
Kpx = 0.25
Kpy = 0.25
Kdx = 0.25
Kdy = 0.25
Kix = 0
Kiy = 0

# Routine called by C program.
def action(img_bytes, img_width, img_height, is_belly, ctrl_state, vbat_flying_percentage, theta, phi, psi, altitude, vx, vy, vz):
        import pdb; pdb.set_trace()
        print "autopylot taking action!"

	# Set up command defaults
	start = 0 
	select = 0 
	zap = 0
	enable = 0 
	phi = 0 	
	theta = 0 
	gaz = 0
	yaw = 0
	zap = 0

        # Set up state variables first time around
	if not hasattr(action, 'count'):
                action.count = 0
                action.threes_image = cv.CreateImage((img_width,img_height), cv.IPL_DEPTH_8U, 1)  
                cv.Set(action.threes_image, 3)
		action.errx_1 = 0
		action.erry_1 = 0
		action.phi_1 = 0
		action.gaz_1 = 0

	# Create full-color image from bytes
	full_image = create_image_header(img_width, img_height, 3)  
        cv.SetData(full_image, img_bytes, img_width*3)
  
        # Create separate red, green, and blue images from bytes
        r_image = create_grayscale_image(img_bytes, img_width, img_height, 2)
        b_image = create_grayscale_image(img_bytes, img_width, img_height, 0)
        g_image = create_grayscale_image(img_bytes, img_width, img_height, 1)

        # Remove 1/3 of red and blue components from green
        div_and_sub(g_image, r_image, action.threes_image)
        div_and_sub(g_image, b_image, action.threes_image)

        # Threshold and erode green image
	cv.Threshold(g_image, g_image, THRESHOLD, 255, cv.CV_THRESH_BINARY)
	cv.Erode(g_image, g_image)

        # Find centroid of eroded image
        moments = cv.Moments(g_image, 1) # binary flag
        centroid_x = centroid(moments, 1, 0)
        centroid_y = centroid(moments, 0, 1)

	# Use centroid if it exists
        if centroid_x != None and centroid_y != None:

                ctr = (centroid_x, centroid_y)

		# Put black circle in at centroid in image
                cv.Circle(full_image, ctr, 4, (0,0,0))

		# Compute proportional distance (error) of centroid from image center
		errx =  dst(ctr, 0, img_width)
		erry = -dst(ctr, 1, img_height)

		# Compute vertical, horizontal velocity commands based on PID control after first iteration
                if action.count > 0:
                        phi = pid(action.phi_1, errx, action.errx_1, Kpx, Kix, Kdx)
                        gaz = pid(action.gaz_1, erry, action.erry_1, Kpy, Kiy, Kdy)
		        enable = 1

                # Remember PID variables for next iteration
		action.errx_1 = errx
		action.erry_1 = erry
		action.phi_1 = phi
		action.gaz_1 = gaz
                action.count += 1

        # Display full-color image
	cv.NamedWindow(WINDOW_NAME)
        cv.ShowImage(WINDOW_NAME, full_image)

	# Force image display
	cv.WaitKey(5)

        # Send control parameters back to drone
	return (start, select, zap, enable, phi, theta, gaz, yaw)

# Removes 1/3 of other_image from src_dst_image
def div_and_sub(src_dst_image, other_image, threes_image):
        cv.Div(other_image, threes_image, other_image) 
        cv.Sub(src_dst_image, other_image, src_dst_image) 

# Computes centroid based on x, y flags
def centroid(moments, x, y):
        result = None
        moments00 = cv.GetSpatialMoment(moments, 0, 0)
        if moments00 != 0:
                result = int(cv.GetSpatialMoment(moments, x, y) / moments00)
        return result

# Creates grayscale image from bytes
def create_grayscale_image(img_bytes, img_width, img_height, channel):
        image = create_image_header(img_width, img_height, 1)
        cv.SetData(image, img_bytes[channel::3], img_width)
        return image

# Creates image header from bytes
def create_image_header(img_width, img_height, channels):
	return cv.CreateImageHeader((img_width,img_height), cv.IPL_DEPTH_8U, channels)  

# Simple PID controller from http://www.control.com/thread/1026159301
def pid(out_1, err, err_1, Kp, Ki, Kd):
       return Kp*err + Ki*(err+err_1) + Kd*(err-err_1) 

# Returns proportional distance to image center along specified dimension.
# Above center = -; Below = +
# Right of center = +; Left = 1
def dst(ctr, dim, siz):
    siz = siz/2
    return (ctr[dim] - siz) / float(siz)
