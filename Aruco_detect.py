import serial
import time
import cv2 as cv
import numpy as np
from cv2 import aruco

# Serial communication settings (modify for your ports)
#servo_port = 'COM9'


# Servo control function (replace with your specific servo library commands)
# AR marker detection and communication
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
param_markers = aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(marker_dict, param_markers)

cap = cv.VideoCapture(0)

# Serial connection for communication with Arduino
ser = serial.Serial('COM7', 9600 ,timeout=1)

# Function to control servo motors based on marker position
def rotate_servo(x, y):
    val = int(x)
    val2 = int(y)
    
    ser.write(f"{val},{val2}\n".encode())

# Initial angle for servo motor
thetai = 90

while True:
    # Read frame from camera
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to grayscale
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # Detect markers in the frame
    marker_corners, marker_IDs, reject = detector.detectMarkers(gray_frame)

    if marker_corners:
        for ids, corners in zip(marker_IDs, marker_corners):
            # Draw marker outline on frame
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 0, 255), 4, cv.LINE_AA
            )
            # Calculate marker center coordinates
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_left = corners[0].ravel()
            top_right = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()
            center = [(top_left[0] + bottom_right[0]) / 2, (top_left[1] + bottom_right[1]) / 2]
            x = center[0].astype(np.int32) 
            y = center[1].astype(np.int32)
            x1= x*(9/32)
            x2 = x1.astype(np.int32) 
            theta2x=int(thetai+ (90-x1)*0.17)

            y1=y*(9/24)
            y2=y1.astype(np.int32)
            theta2y=int(thetai+ (90-y1)*0.15)
            
            # Display marker ID and center coordinates on frame
            cv.putText(
                frame,
                f"id: {ids} (x,y):{x2}",
                [x, y],
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (0, 0, 255),
                2,
                cv.LINE_AA,
            )
            
            # Rotate servo motors based on marker position
            rotate_servo(theta2x,theta2y)
            
            # Update initial angle if marker position is close to desired
            if abs(90-x1) <= 2:
                thetai = theta2x
            print(theta2x,thetai)
            # Check for a specific marker ID to terminate (replace with your termination logic)
            # if ids == 10:  # Replace 10 with the desired marker ID for termination
            #    print('Finished program')
            #    break
            
    # Display the frame
    cv.imshow("frame", frame)

    # Check for key press to exit
    key = cv.waitKey(10)
    if key == ord("q"):
        break

# Release the camera and close serial connection
cap.release()
ser.close() 
cv.destroyAllWindows()
