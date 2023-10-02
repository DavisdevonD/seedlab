from time import sleep
import numpy as np
import cv2
from cv2 import aruco
import os
from smbus2 import SMBus
import smbus2
import board
import time
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# Setup lcd screen junk
i2c = board.I2C()
lcd_columns = 16
lcd_rows = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()
lcd.color = [100, 0, 0]

# Initialize the I2C bus
bus = SMBus(1)

#Initialize camera
camera = cv2.VideoCapture(0)
sleep(0.1) #Sleep to allow cam to initialize

#Aruco detection dictionary and params setup
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
parameters = aruco.DetectorParameters()


camheight = 480
camwidth = 640

def find_center_of_corner(corner):
	for vector in corner: #Find the center of 4 vectors by averaging the coordinates of the vectors.
		xavg = (vector[0, 0] + vector[1, 0] + vector[2, 0] + vector[3, 0])/4.0
		yavg = (vector[0, 1] + vector[1, 1] + vector[2, 1] + vector[3, 1])/4.0
		#print("X: ", xavg, " Y: ", yavg)
		return [xavg, yavg]

def send_int_to_arduino(value): #Send an int to the arduino
    bus = smbus2.SMBus(1)   # Initializes I2C bus
    
    ARDUINO_ADDRESS = 0x04  # Address we set for Arduino
    if 0 <= value <= 255:   # The I2C protocol
        try:
            bus.write_byte(ARDUINO_ADDRESS, value)
            print(f"Sent {value} to Arduino")
        except Exception as e:
            print(f"Error sending {value} to Arduino: {e}")
        finally:
            bus.close()
    else:
        print("Value out of byte range")

#-----SUPERLOOP-----
while True:
	ret, image = camera.read()

	if ret:
		#COMP VIS STUFF
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #convert to gray for computer vision processing
		corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters) #detect aruco markers
		image = aruco.drawDetectedMarkers(image.copy(), corners, ids) #Draw boxes around detected markers
		cv2.imshow("Image", image) #Draw the image
		

		if corners: #if atleast 1 aruco marker detected 
			center = find_center_of_corner(corners[0]) 
			center_w = [center[0] - (camwidth/2), -(center[1] - (camheight/2))] #Determine the center of the aruco in relation to center of image
			xneg = (np.sign(center_w[0]) == -1)		# Determine x sign
			yneg = (np.sign(center_w[1]) == -1)     # Determine y sign
			xpos = not xneg #invert to make next part easier
			ypos = not yneg
			result = -1 #Default
			if (xpos and ypos): #Determine resulting quadrant(0 is upper right, continues clockwise)
				result = 0
			if (xpos and yneg):
				result = 1
			if (xneg and yneg):
				result = 2
			if (xneg and ypos):
				result = 3
			print(result)
			send_int_to_arduino(result) #Send location to arduino
			lcd.message = str(result) #Set LCD message
			
	

        #DEPRECATED
		# print("Image dimensions: ", image.shape)
		#[left, right] = np.split(image, 2, axis=1)
		#[upperleft, lowerleft] = np.split(left, 2, axis=0)
		#[upperright, lowerright] = np.split(right, 2, axis=0)
		#cv2.imshow("Left", left)
		#cv2.imshow("Right", right)
		
		#urcorners, urids, _ = aruco.detectMarkers(cv2.cvtColor(upperright, cv2.COLOR_BGR2GRAY), aruco_dict, parameters=parameters)
		#ulcorners, ulids, _ = aruco.detectMarkers(cv2.cvtColor(upperleft, cv2.COLOR_BGR2GRAY), aruco_dict, parameters=parameters)
		#llcorners, llids, _ = aruco.detectMarkers(cv2.cvtColor(lowerleft, cv2.COLOR_BGR2GRAY), aruco_dict, parameters=parameters)
		#lrcorners, lrids, _ = aruco.detectMarkers(cv2.cvtColor(lowerright, cv2.COLOR_BGR2GRAY), aruco_dict, parameters=parameters)
		
		#upperright = aruco.drawDetectedMarkers(upperright.copy(), urcorners, urids)
		#upperleft = aruco.drawDetectedMarkers(upperleft.copy(), ulcorners, ulids)
		#lowerleft = aruco.drawDetectedMarkers(lowerleft.copy(), llcorners, llids)
		#lowerright = aruco.drawDetectedMarkers(lowerright.copy(), lrcorners, lrids)
		#cv2.imshow("Upper left", upperleft)
		#cv2.imshow("Lower left", lowerleft)
		#cv2.imshow("Upper right", upperright)
		#cv2.imshow("Lower right", lowerright)

		
		if cv2.waitKey(1) & 0xFF == ord('q'): #Key to kill imshow window
			break
	else:
		print("ERROR, CAMERA FEED FAILED")
		quit()
camera.release() #Release camera
cv2.destroyAllWindows() #Destroy display windows

