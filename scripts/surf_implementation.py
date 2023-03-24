import cv2
from cv2 import xfeatures2d
import numpy as np
import yaml

# Create SURF detector object
surf = cv2.xfeatures2d.SURF_create()
surf.setHessianThreshold(1000)


# Create video capture object
cap = cv2.VideoCapture(2)

# Create feature matcher
matcher = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)

with open('/home/exmachina/NEU/SEM-2/PRCV/Assignment/Assgn_4/CameraCalibration_AugmentedReality/calib_params/my_webcam_Calibration_Parameters.yaml', 'r') as file:
    calib_data = yaml.safe_load(file)

cp = calib_data['camera_matrix']['data']

K = np.array([[cp[0], cp[1], cp[2]],[cp[3], cp[4],cp[5]], [cp[6], cp[7], cp[8]]])

while True:
    # Read a frame from the video stream
    ret, frame = cap.read()
    if not ret:
        continue

    # frame = cv2.imread('/home/exmachina/NEU/SEM-2/PRCV/Assignment/Assgn_4/CameraCalibration_AugmentedReality/pattern2.jpg')

    # Detect SURF features in the frame
    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    kp1, des1 = surf.detectAndCompute(frame, None)

    # Draw the detected features on the frame
    frame = cv2.drawKeypoints(frame, kp1, frame, color = (50,200,50),flags=cv2.DRAW_MATCHES_FLAGS_DEFAULT)

    # Display the frame
    cv2.imshow('SURF Features', frame)
    # Wait for key press to exit
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

# Release video capture object and close all windows
cap.release()
cv2.destroyAllWindows()