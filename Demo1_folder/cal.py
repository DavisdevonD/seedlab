import numpy as np
import cv2

# Checkerboard dimensions
CHECKERBOARD = (7, 7)
square_size = 1.6  

#detecting checkerboard corners
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Error: Could'nt open camera")
    exit()

num_images_captured = 0
objpoints = []  # 3D points
imgpoints = []  # 2D points 

# Prepare object points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * square_size

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    # Only draw if corners are detected
    if ret:
        cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, ret)

    cv2.imshow('Calibration', frame)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord('c'):
        if ret:
            cv2.imwrite(f"calibration_{num_images_captured}.jpg", frame)
            num_images_captured += 1
            print(f"Captured image {num_images_captured}")

            # Store the detected corners and corresponding 3D points
            objpoints.append(objp)
            imgpoints.append(corners)
        else:
            print("Corners not detected, try again.")
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# Camera calibration
if len(objpoints) > 0 and len(imgpoints) > 0:
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    print("Calibration successful")
    print("Camera Matrix:")
    print(camera_matrix)
    print("Distortion Coefficients:")
    print(dist_coeffs)
else:
    print("No valid chessboard patterns detected in any of the images.")
