import cv2
import numpy as np
import cvlib as cv

def find_plant_pixels(image_path):
    # Load the image
    image = cv2.imread(image_path)
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Define the HSV range for green (adjust as needed)
    lower_green = np.array([35, 40, 40])
    upper_green = np.array([85, 255, 255])
    
    # Create a mask that only includes green pixels
    mask = cv2.inRange(hsv_image, lower_green, upper_green)
    
    # Find coordinates where mask is non-zero (i.e., plant pixels)
    plant_pixels = np.column_stack(np.where(mask > 0))
    
    return plant_pixels

# Example usage
image_path = 'Mouse1.jpg'
plant_pixels = find_plant_pixels(image_path)
print("Found plant pixels at:", plant_pixels)