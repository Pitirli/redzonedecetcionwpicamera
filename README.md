from picamera2 import Picamera2  # Import the Picamera2 library for capturing frames from the Raspberry Pi camera
import cv2  # OpenCV for image processing
from PIL import Image  # PIL for image manipulation
import numpy as np  # NumPy for numerical operations
import RPi.GPIO as GPIO  # RPi.GPIO for controlling GPIO pins on the Raspberry Pi
from time import sleep  # Sleep function for introducing delays

# Setup the GPIO mode to use the board pin numbers
GPIO.setmode(GPIO.BOARD)

# Function to control PWM (Pulse Width Modulation) on a specified pin
def pwm_control(pin, frequency, duty_cycle_start, duty_cycle_end, delay):
    GPIO.setup(pin, GPIO.OUT)  # Set the specified pin as an output pin
    p = GPIO.PWM(pin, frequency)  # Initialize PWM on the specified pin with the given frequency
    p.start(0)  # Start PWM with a duty cycle of 0%

    # Change duty cycle to the start value, wait for the specified delay, then change to the end value
    p.ChangeDutyCycle(duty_cycle_start)
    sleep(delay)
    p.ChangeDutyCycle(duty_cycle_end)
    sleep(delay)

    # Stop the PWM and clean up the GPIO setup
    p.stop()
    GPIO.cleanup()

# Define red color in BGR format for later use
red = [0, 0, 255]

# Initialize the Picamera2 object to capture frames
picam2 = Picamera2()

# Configure the camera with the preview settings (resolution: 640x480, format: XRGB8888)
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()  # Start the camera

try:
    while True:
        # Capture a frame from the camera as an array
        frame = picam2.capture_array()

        # Convert the captured frame from BGR color space to HSV color space
        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds for red color in the HSV color space (first range)
        lowerLimit0 = np.array([0, 120, 70], dtype=np.uint8)  # Lower bound for red hue
        upperLimit0 = np.array([10, 255, 255], dtype=np.uint8)  # Upper bound for red hue
        mask0 = cv2.inRange(hsvImage, lowerLimit0, upperLimit0)  # Create a mask for the first red range

        # Define the lower and upper bounds for red color in the HSV color space (second range)
        lowerLimit1 = np.array([170, 120, 70], dtype=np.uint8)  # Lower bound for red hue
        upperLimit1 = np.array([180, 255, 255], dtype=np.uint8)  # Upper bound for red hue
        mask1 = cv2.inRange(hsvImage, lowerLimit1, upperLimit1)  # Create a mask for the second red range

        # Combine the two masks to detect the full red color range
        mask = mask0 + mask1

        # Apply erosion and dilation to clean up the noise in the mask
        mask = cv2.erode(mask, None, iterations=2)  # Erode the mask to remove small white regions
        mask = cv2.dilate(mask, None, iterations=2)  # Dilate the mask to restore object size

        # Find the contours of the red object(s) detected in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # If there are any contours detected
        if contours:
            # Find the largest contour by area
            largest_contour = max(contours, key=cv2.contourArea)

            # Get the bounding box coordinates around the largest contour
            (x, y, w, h) = cv2.boundingRect(largest_contour)

            # Draw a red rectangle around the largest contour
            frame = cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)

            # Calculate the center of the contour using moments
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])  # Calculate the X coordinate of the center
                cY = int(M["m01"] / M["m00"])  # Calculate the Y coordinate of the center

                # Draw a green circle at the center of the contour
                cv2.circle(frame, (cX, cY), 5, (0, 255, 0), -1)

            # Call the PWM control function to control a motor or other component
            pwm_control(11, 50, 3, 12, 1)

        # Display the frame with the rectangle and circle overlays
        cv2.imshow('frame', frame)

        # Break the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop the camera and clean up the resources
    picam2.stop()
    cv2.destroyAllWindows()  # Close all OpenCV windows
