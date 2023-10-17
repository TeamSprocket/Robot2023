import cv2
import numpy as np

# Define the lower and upper bounds for the purple color in HSV format
lower_purple = np.array([125, 50, 50])
upper_purple = np.array([160, 255, 255])

# Load the video feed or an image
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # You can replace 0 with the video file path if using a video


def runPipeline(frame, llrobot):
    # Constants
    minHSV = (125, 50, 50)
    maxHSV = (160, 255, 255)
    percentThreshold = 0.01
    imgW, imgH = 640, 480

    # Get Contours
    img = cv2.resize(frame, (imgW, imgH), interpolation=cv2.INTER_LINEAR)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_threshold = cv2.inRange(img_hsv, minHSV, maxHSV)
    contours, _ = cv2.findContours(img_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Prepare return data
    llpython = [0, 0, 0, 0, 0, 0, 0, 0]
    largestContour = np.array([[]])
    if len(contours) > 0 and cv2.contourArea(max(contours, key=cv2.contourArea)) >= (imgW * imgH * percentThreshold):
        # print(contours)
        # Draw contours
        cv2.drawContours(img, contours, -1, 255, 2)
        largestContour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largestContour)
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.circle(img, (int(x + (w / 2)), int(y + (h / 2))), 5, (255, 0, 0), 2)

        # record some custom data to send back to the robot
        llpython = [1, x + (w / 2), y + (h / 2), 0, 0, 0, 0, 0]

    # Return Data
    return largestContour, img, llpython


while True:
    _, frame = cap.read()
    ct, img, arr = runPipeline(frame, None)
    y, x, _ = img.shape
    cv2.imshow("Window", img)
    if cv2.waitKey(33) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()



