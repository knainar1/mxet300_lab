# L3_warehouse_color_detection.py
# This program was designed to identify four colored targets using a SCUTTLE USB camera.
# The detected color and the center of the target are displayed on the camera image.

import cv2                  # For image capture and processing
import numpy as np          # For image arrays and morphology
import netifaces as ni      # For getting the IP address for the camera stream
from time import sleep, monotonic


# Gets IP to grab MJPG stream
def getIp():
    for interface in ni.interfaces()[1:]:       # For interfaces eth0 and wlan0

        try:
            ip = ni.ifaddresses(interface)[ni.AF_INET][0]['addr']
            return ip

        except (KeyError, IndexError):           # This interface does not have IPv4 information
            continue                             # Try the next interface

    return 0


# Camera
stream_ip = getIp()
camera_input = None

if stream_ip:
    camera_input = 'http://' + stream_ip + ':8090/?action=stream'   # Address for stream

size_w = 240       # Resized image width in pixels
size_h = 160       # Resized image height in pixels


# Color Ranges, described in HSV
# OpenCV Hue values range from 0 to 179. Saturation and Value range from 0 to 255.

# Pink
pink_v1_min = 130       # Minimum H value
pink_v2_min = 95        # Minimum S value
pink_v3_min = 205       # Minimum V value
pink_v1_max = 179       # Maximum H value (OpenCV maximum is 179)
pink_v2_max = 255       # Maximum S value
pink_v3_max = 255       # Maximum V value

# Yellow
yellow_v1_min = 35      # Minimum H value
yellow_v2_min = 100     # Minimum S value
yellow_v3_min = 150     # Minimum V value
yellow_v1_max = 55      # Maximum H value
yellow_v2_max = 255     # Maximum S value
yellow_v3_max = 255     # Maximum V value

# Orange
orange_v1_min = 0       # Minimum H value
orange_v2_min = 120     # Minimum S value
orange_v3_min = 230     # Minimum V value
orange_v1_max = 155     # Maximum H value (kept at the tested value)
orange_v2_max = 175     # Maximum S value
orange_v3_max = 255     # Maximum V value

# Green
green_v1_min = 60       # Minimum H value
green_v2_min = 145      # Minimum S value
green_v3_min = 180      # Minimum V value
green_v1_max = 85       # Maximum H value
green_v2_max = 255      # Maximum S value
green_v3_max = 255      # Maximum V value


min_area = 100          # Minimum contour area to ignore small objects/noise
kernel = np.ones((5, 5), np.uint8)   # Set kernel size for noise removal


def main():
    # Try opening camera with default method
    camera = cv2.VideoCapture(0)

    # Try opening camera stream if default camera failed
    if not camera.isOpened():
        camera.release()

        if camera_input:
            camera = cv2.VideoCapture(camera_input)

    if not camera.isOpened():
        camera.release()
        print("Failed to open camera!")
        return

    camera.set(3, size_w)       # Set width of images retrieved from camera
    camera.set(4, size_h)       # Set height of images retrieved from camera

    last_report = 0.0

    try:
        while True:
            sleep(.05)

            ret, image = camera.read()      # Get image from camera

            # Make sure image was grabbed
            if not ret:
                print("Failed to retrieve image!")
                break

            image = cv2.resize(image, (size_w, size_h))
            image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)    # Convert image to HSV

            height, width, channels = image.shape             # Get shape of image

            # Find all pixels in each color range
            thresh_pink = cv2.inRange(image,
                (pink_v1_min, pink_v2_min, pink_v3_min),
                (pink_v1_max, pink_v2_max, pink_v3_max))

            thresh_yellow = cv2.inRange(image,
                (yellow_v1_min, yellow_v2_min, yellow_v3_min),
                (yellow_v1_max, yellow_v2_max, yellow_v3_max))

            thresh_orange = cv2.inRange(image,
                (orange_v1_min, orange_v2_min, orange_v3_min),
                (orange_v1_max, orange_v2_max, orange_v3_max))

            thresh_green = cv2.inRange(image,
                (green_v1_min, green_v2_min, green_v3_min),
                (green_v1_max, green_v2_max, green_v3_max))

            # Pair each threshold with its color name and display color (BGR)
            color_thresholds = [
                ("PINK", thresh_pink, (255, 0, 255)),
                ("YELLOW", thresh_yellow, (0, 255, 255)),
                ("ORANGE", thresh_orange, (0, 165, 255)),
                ("GREEN", thresh_green, (0, 255, 0))
            ]

            # Convert the HSV image back to BGR so boxes and labels can be drawn
            display_image = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
            detected_colors = []

            for color, thresh, box_color in color_thresholds:
                mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)   # Remove noise
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)   # Fill small openings

                cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)[-2]                         # Find closed shapes

                if len(cnts):
                    c = max(cnts, key=cv2.contourArea)                  # Find largest target area
                    area = cv2.contourArea(c)

                    if area >= min_area:
                        x, y, w, h = cv2.boundingRect(c)                 # Bounding rectangle
                        center = (int(x + 0.5 * w), int(y + 0.5 * h))   # Center of target

                        cv2.rectangle(display_image, (x, y), (x + w, y + h), box_color, 2)
                        cv2.circle(display_image, center, 3, box_color, -1)
                        cv2.putText(display_image, color + " " + str(center),
                            (x, max(y - 8, 15)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.4, box_color, 1)

                        detected_colors.append(color + " center=" + str(center))

            # Report detections in the terminal once per second
            if monotonic() - last_report >= 1.0:
                if detected_colors:
                    print("Detected: " + " | ".join(detected_colors))
                else:
                    print("No targets detected")
                last_report = monotonic()

            cv2.imshow("Warehouse Color Detection", display_image)

            # Press 'q' to exit the program
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    except KeyboardInterrupt:      # Catch a Ctrl-C event and exit cleanly
        pass

    finally:
        camera.release()
        cv2.destroyAllWindows()
        print("Exiting Warehouse Color Detection.")


if __name__ == '__main__':
    main()
