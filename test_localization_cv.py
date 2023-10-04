import cv2
import numpy as np
import os

def process_image(image):
    min_area = 10_000

    # define the alpha and beta
    alpha = 1.8 # Contrast control
    beta = 10 # Brightness control

    # call convertScaleAbs function
    image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)

    # Preprocess the image (e.g., convert to grayscale)
    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Define color ranges for yellow and red
    lower_yellow = np.array([0, 100, 100])
    upper_yellow = np.array([200, 255, 255])

    lower_red = np.array([0, 0, 100])
    upper_red = np.array([200, 200, 255])

    # Create masks for yellow and red
    mask_yellow = cv2.inRange(image, lower_yellow, upper_yellow)
    mask_red = cv2.inRange(image, lower_red, upper_red)

    # Find contours in the masks
    contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours (e.g., by area)
    filtered_contours_yellow = [c for c in contours_yellow if cv2.contourArea(c) > min_area]
    filtered_contours_red = [c for c in contours_red if cv2.contourArea(c) > min_area]

    # Get object positions (centroids)
    yellow_positions = [cv2.moments(c) for c in filtered_contours_yellow]
    red_positions = [cv2.moments(c) for c in filtered_contours_red]

    # Calculate centroids
    yellow_centroids = [(int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])) for m in yellow_positions]
    red_centroids = [(int(m['m10'] / m['m00']), int(m['m01'] / m['m00'])) for m in red_positions]

    # Print the centroids
    print("Yellow Object Positions:", yellow_centroids)
    print("Red Object Positions:", red_centroids)

    return image, filtered_contours_red, filtered_contours_yellow, yellow_centroids, red_centroids

os.mkdir(os.path.join(os.getcwd(), "result"))

cap = cv2.VideoCapture(4)

def make_1080p():
    cap.set(3, 1920)
    cap.set(4, 1080)

# Check if the webcam is opened correctly
if not cap.isOpened():
    raise IOError("Cannot open webcam")
else:
    make_1080p()

cnt = 0

while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    image, filtered_contours_red, filtered_contours_yellow, yellow_centroids, red_centroids = process_image(frame)

    # if len(yellow_centroids) != 2 or len(red_centroids) != 2:
    #     continue

    # Draw contours and centroids on the image for visualization
    cv2.drawContours(image, filtered_contours_yellow, -1, (255, 0, 0), 2)
    cv2.drawContours(image, filtered_contours_red, -1, (255, 255, 0), 2)

    for centroid in yellow_centroids:
        cv2.circle(image, centroid, 5, (0, 0, 255), -1)

    for centroid in red_centroids:
        cv2.circle(image, centroid, 5, (255, 0, 0), -1)

    # Display the image
    cv2.imshow('Segmented Image', image)

    c = cv2.waitKey(1)
    if c == 27:
        break

    cv2.imwrite(os.path.join(os.getcwd(), "result", "{}.png".format(cnt)), image)
    cnt+=1

cap.release()
cv2.destroyAllWindows()