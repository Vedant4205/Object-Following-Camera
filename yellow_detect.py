import cv2
import numpy as np
from matplotlib import pyplot as plt
import random
from collections import deque
import math
import serial
import threading
import time

# Open video capture
cap = cv2.VideoCapture(0)

# Serial communication with Arduino
ser = serial.Serial('COM7', 9600, timeout=1) 

# Function to rotate servo motor based on x and y coordinates
def rotate_servo(x, y):
    val = int(x)
    val2 = int(y)
    ser.write(f"{val},{val2}\n".encode())


# Initial angles for servo motors
thetaix = 90
thetai = 90

# Flags to control thread creation
c = False
m = False

while True:
    # Read and flip frame
    _, frame = cap.read()
    frame = cv2.flip(frame, 1)

    # Blur the frame a little
    blur_frame = cv2.GaussianBlur(frame, (7, 7), 0)

    # Convert from BGR to HSV color format
    hsv = cv2.cvtColor(blur_frame, cv2.COLOR_BGR2HSV)

    # Define lower and upper range of hsv color to detect. Blue here
    lower_blue = np.array([20, 100, 100])
    upper_blue = np.array([40, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Make elliptical kernel
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))

    # Opening morph(erosion followed by dilation)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Find all contours
    contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2:]

    if len(contours) > 0:
        # Find the biggest contour
        biggest_contour = max(contours, key=cv2.contourArea)

        # Find center of contour and draw filled circle
        moments = cv2.moments(biggest_contour)
        centre_of_contour = (int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00']))
        cv2.circle(frame, centre_of_contour, 5, (0, 0, 255), -1)
        
        # Bound the contour with ellipse
        ellipse = cv2.fitEllipse(biggest_contour)
        x2 = int((moments['m10'] / moments['m00']) * (9 / 32))
        y2 = int((moments['m01'] / moments['m00']) * (9 / 24))
        
        # Convert centroid coordinates to servo angles
        theta2x = 170 - int(thetaix + (90 - x2) * 0.17) 
        theta2y = 170 - int(thetai + (90 - y2) * 0.15)
        
        # Limit servo angles
        if theta2x >= 170:
            theta2x = 170
        if theta2x <= 0:
            theta2x = 0
        if theta2y >= 170:
            theta2y = 170
        if theta2y <= 0:
            theta2y = 0
        
        # Rotate servo
        rotate_servo(theta2x, theta2y)

        # Create a thread to update thetaix variable
        class ThreadJob(threading.Thread):
            def _init_(self, callback, event, interval):
                self.callback = callback
                self.event = event
                self.interval = interval
                super(ThreadJob, self)._init_()

            def run(self):
                while not self.event.wait(self.interval):
                    self.callback()
        event = threading.Event()
        def foo():
            global thetaix
            global thetai
            thetaix = 170 - theta2x
            print(theta2x, thetaix)
        k = ThreadJob(foo, event, 0.1)
        if c == False:
            k.start()
            c = True
        
        # Create a thread to update thetai variable
        class ThreadJob1(threading.Thread):
            def _init_(self, callback, event, interval):
                self.callback = callback
                self.event = event
                self.interval = interval
                super(ThreadJob1, self)._init_()

            def run(self):
                while not self.event.wait(self.interval):
                    self.callback()
        event2 = threading.Event()
        def foo1():
            global thetaix
            global thetai
            thetai = 170 - theta2y
        k2 = ThreadJob1(foo1, event2, 0.07)
        if m == False:
            k2.start()
            m = True
       

    # Display original and mask frames
    cv2.imshow('original', frame)
    cv2.imshow('mask', mask)

    # Check for key press
    k = cv2.waitKey(5) & 0xFF
    if k == 27:  # ESC key
        break

# Close all windows, release serial connection and video capture
cv2.destroyAllWindows()
ser.close()
cap.release()