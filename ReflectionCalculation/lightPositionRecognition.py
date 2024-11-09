import cv2
import os
import numpy as np

def change_to_script_directory():
    # Get the directory of the currently running script
    script_directory = os.path.dirname(os.path.abspath(__file__))
    
    # Change the current working directory to the script's directory
    os.chdir(script_directory)

change_to_script_directory()

# Load the images
imageA1 = cv2.imread('noLight_scenario1.png')  # Image without sunlight
imageB1 = cv2.imread('yesLight_scenario1.png')  # Image with sunlight

imageA2 = cv2.imread('noLight_scenario2.png')  # Image without sunlight
imageB2 = cv2.imread('yesLight_scenario2.png')  # Image with sunlight

imageA3 = cv2.imread('noLight_scenario3.jpg')  # Image without sunlight
imageB3 = cv2.imread('yesLight_scenario3.jpg')  # Image with sunlight

# Prompt the user for input
while True:
    try:
        user_input = int(input("Please choose a scenario (1, 2, or 3): "))
        if user_input not in [1, 2, 3]:
            raise ValueError("Invalid input. Please enter 1, 2, or 3.")
        break
    except ValueError as e:
        print(e)

# Based on the user's choice, select the corresponding images
if user_input == 1:
    imageA, imageB = imageA1, imageB1
elif user_input == 2:
    imageA, imageB = imageA2, imageB2
else:
    imageA, imageB = imageA3, imageB3

# Convert the images to grayscale
grayA = cv2.cvtColor(imageA, cv2.COLOR_BGR2GRAY)
grayB = cv2.cvtColor(imageB, cv2.COLOR_BGR2GRAY)

# Show the original images
cv2.imshow("Image without sunlight", imageA)
cv2.waitKey(0)
cv2.imshow("Image with sunlight", imageB)
cv2.waitKey(0)

# Show the original images
cv2.imshow("Image without sunlight- Gray Scale", grayA)
cv2.waitKey(0)
cv2.imshow("Image with sunlight - Gray Scale", grayB)
cv2.waitKey(0)

# Compute the absolute difference between the two grayscale images
diff = cv2.absdiff(grayA, grayB)

# Show the grayscale difference image
cv2.imshow("Difference Image (Grayscale)", diff)
cv2.waitKey(0)

# Apply thresholding to the difference image
_, thresh = cv2.threshold(diff, 50, 255, cv2.THRESH_BINARY)

# Show the thresholded image
cv2.imshow("Thresholded Image", thresh)
cv2.waitKey(0)

white_pixels = np.column_stack(np.where(thresh == 255))

# Check if there are any white pixels
if white_pixels.size > 0:
    # Calculate the average x and y coordinates
    reflection_x = int(np.mean(white_pixels[:, 1]))  # x-coordinates are in the second column
    reflection_y = int(np.mean(white_pixels[:, 0]))  # y-coordinates are in the first column
    print(f"Average position of reflection: X = {reflection_x} | Y = {reflection_y}")
else:
    print("No white pixels found in the image.")

radius = 10


output_image = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)  # Convert to BGR to draw colored circle
cv2.circle(output_image, (reflection_x, reflection_y), radius, (0, 0, 255), 2)  # Red circle with thickness 2

# Display the image (if running in a local environment with display capabilities)
cv2.imshow("Image with Circle", output_image)
cv2.waitKey(0)
# Close all windows
cv2.destroyAllWindows()
