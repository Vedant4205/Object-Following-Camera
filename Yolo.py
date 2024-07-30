from ultralytics import YOLO
import cv2
import numpy as np
import threading
import serial

# Load YOLOv8 model
model = YOLO("yolov8x.pt")

# Open video capture
cap = cv2.VideoCapture(0)



ser = serial.Serial('COM7', 9600, timeout=1) 

def rotate_servo(x, y):
    val = int(x)
    val2 = int(y)
    ser.write(f"{val},{val2}\n".encode())

D = 0.15

thetaix = 90
thetai = 90

c = False
m = False
x_f, y_f = int((cap.get(cv2.CAP_PROP_FRAME_WIDTH)) / 2), int((cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) / 2)

while True:
    # Read frame from the video
    ret, frame = cap.read()
    if ret == True:
        cv2.circle(frame, (x_f, y_f), 3, (0, 255, 0), -1)

        # Perform object detection on the frame
        results = model(frame)

        # Extract object information from the results
        result = results[0]

        # Initialize variables to track the largest person
        max_area = 0
        max_confidence = 0
        max_box = None

        # Loop through detected objects
        for box in result.boxes:
            if result.names[box.cls[0].item()] == "cell phone":
                # Calculate area of the bounding box
                area = (box.xyxy[0][2] - box.xyxy[0][0]) * (box.xyxy[0][3] - box.xyxy[0][1])

                # Update max_area and max_box if current area is larger
                if area > max_area and box.conf[0].item() > max_confidence:
                    max_area = area
                    max_confidence = box.conf[0].item()
                    max_box = box

        if max_box is not None:
            # Extract coordinates of the largest person
            x_min, y_min, x_max, y_max = [round(x) for x in max_box.xyxy[0].tolist()]

            # Draw bounding box rectangle
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)

            # Display class label and confidence
            label = f"{result.names[max_box.cls[0].item()]}: {round(max_confidence, 2)}"
            cv2.putText(frame, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Calculate centroid of the largest person
            x_c, y_c = int((x_min + x_max) / 2), int((y_min + y_max) / 2)
            cv2.circle(frame, (x_c, y_c), 3, (255, 0, 0), -1)

            # Convert centroid coordinates to servo angles
            x2 = x_c * (9 / 32)
            y2 = y_c * (9 / 24)
            theta2x = int(thetaix + (90 - x2) * 0.17)
            theta2y =170-int(thetai + (90 - y2) * 0.15)

            # Limit servo angles
            theta2x = max(0, min(theta2x, 170))
            theta2y =max(0, min(theta2y, 170))

            # Rotate servo
            rotate_servo(theta2x, theta2y)
            
            class ThreadJob(threading.Thread):
                def _init_(self,callback,event,interval):
                    '''runs the callback function after interval seconds
                    :param callback:  callback function to invoke
                    :param event: external event for controlling the update operation
                    :param interval: time in seconds after which are required to fire the callback
                    :type callback: function
                    :type interval: int
                    '''
                    self.callback = callback
                    self.event = event
                    self.interval = interval
                    super(ThreadJob,self)._init_()

                def run(self):
                    while not self.event.wait(self.interval):
                        self.callback()
            event = threading.Event()
            def foo():
                global thetaix
                global thetai
                thetaix=theta2x
                print(theta2x,thetaix)
            k = ThreadJob(foo,event,0.1)
            if c==False:
                k.start()
                c=True

                
            class ThreadJob1(threading.Thread):
                def _init_(self,callback,event,interval):
                    '''runs the callback function after interval seconds
                    :param callback:  callback function to invoke
                    :param event: external event for controlling the update operation
                    :param interval: time in seconds after which are required to fire the callback
                    :type callback: function
                    :type interval: int
                    '''
                    self.callback = callback
                    self.event = event
                    self.interval = interval
                    super(ThreadJob1,self)._init_()

                def run(self):
                    while not self.event.wait(self.interval):
                        self.callback()
            event2 = threading.Event()
            def foo1():
                global thetaix
                global thetai
                thetai=170-theta2y
            k2 = ThreadJob1(foo1,event2,0.07)
            if m==False:
                k2.start()
                m=True
        # Display annotated frame with bounding box
        cv2.imshow('Object Detection', frame)

        # Break the loop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release video capture and close windows
cap.release()
cv2.destroyAllWindows()