import numpy as np
import cv2
from cv2 import aruco
from smbus2 import SMBus
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# Setup LCD
i2c = board.I2C()
lcd_columns = 16
lcd_rows = 2
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()
lcd.color = [100, 0, 0]

# Initialize the I2C bus
bus = SMBus(1)

# Initialize camera
camera = cv2.VideoCapture(0)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
parameters = aruco.DetectorParameters()

#camera peramitors
camheight = 480
camwidth = 640

#Camera calibration parameters
camera_matrix = np.array([
    [680.8518236, 0., 325.94011099],
    [0., 679.92512628, 239.32715234],
    [0., 0., 1.]
])

distortion_coefficients = np.array([
    [0.140801637, -0.644815374, 0.00193957217, 0.000286739249, -0.243970465]
])

#finds center of marker
def find_center_of_corner(corner):
    xavg = np.mean(corner[0][:, 0])
    yavg = np.mean(corner[0][:, 1])
    return [xavg, yavg]

#code left from prior itteration to communicat with arduino
def send_int_to_arduino(value):
    ARDUINO_ADDRESS = 0x04  # Address set for Arduino
    try:
        bus.write_byte(ARDUINO_ADDRESS, value)
        print(f"Sent {value} to Arduino")
    except Exception as e:
        print(f"Error sending {value} to Arduino: {e}")

while True:
    ret, image = camera.read() # reads image from camera

    if ret:
        # Applying camera calibration
        image = cv2.undistort(image, camera_matrix, distortion_coefficients)

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        image = aruco.drawDetectedMarkers(image.copy(), corners, ids)
	
	# finds center of deteted marker 
        if corners:
            center = find_center_of_corner(corners[0])
            cv2.circle(image, (int(center[0]), int(center[1])), 4, (0, 0, 255), -1)  # draw center

            deviation = center[0] - (camwidth/2)
            angle_rad = (np.arctan2(deviation, camwidth / 2)/2)
            angle_deg = np.degrees(angle_rad)

            #print(f"Deviation: {deviation:.2f} pixels, Angle: {angle_deg:.2f} degrees")
            # send_int_to_arduino(angle_rad)  # Uncomment to send data to Arduino
	    #lcd.clear()
            lcd.message = f"Angle: {angle_deg:.2f} deg"
        cv2.imshow("Image", image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("ERROR, CAMERA FEED FAILED")
        quit()

camera.release()
cv2.destroyAllWindows()
