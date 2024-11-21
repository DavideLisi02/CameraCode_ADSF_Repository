import cv2
import numpy as np

class VideoCapture:
    '''
    This class is used to start and update the video stream
    Input:
    name: string, it corresponds to the URL of the used camera
    '''
    def __init__(self, name):
        self.cap = cv2.VideoCapture(name)

    def read(self):
        success, frame = self.cap.read()
        return success, frame
    


def align_images(im1, im2):
    '''
    This function alligns two images. Currently not used.
    '''
    MAX_FEATURES = 500
    GOOD_MATCH_PERCENT = 0.15

    # Convert images to grayscale
    im1Gray = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
    im2Gray = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)
    
    # Detect ORB features and compute descriptors.
    orb = cv2.ORB_create(MAX_FEATURES)
    keypoints1, descriptors1 = orb.detectAndCompute(im1Gray, None)
    keypoints2, descriptors2 = orb.detectAndCompute(im2Gray, None)
    
    # Match features.
    matcher = cv2.DescriptorMatcher_create(cv2.DESCRIPTOR_MATCHER_BRUTEFORCE_HAMMING)
    matches = matcher.match(descriptors1, descriptors2)

    # Ensure `matches` is a list
    if isinstance(matches, tuple):
        matches = list(matches)

    # Sort matches by score
    matches.sort(key=lambda x: x.distance, reverse=False)
    
    # Remove not so good matches
    numGoodMatches = int(len(matches) * GOOD_MATCH_PERCENT)
    matches = matches[:numGoodMatches]
    
    # Draw top matches
    imMatches = cv2.drawMatches(im1, keypoints1, im2, keypoints2, matches, None)
    cv2.imwrite("matches.jpg", imMatches)
    
    # Extract location of good matches
    points1 = np.zeros((len(matches), 2), dtype=np.float32)
    points2 = np.zeros((len(matches), 2), dtype=np.float32)
    
    for i, match in enumerate(matches):
        points1[i, :] = keypoints1[match.queryIdx].pt
        points2[i, :] = keypoints2[match.trainIdx].pt
    
    # Find homography
    h, mask = cv2.findHomography(points1, points2, cv2.RANSAC)
    
    # Use homography
    height, width, channels = im2.shape
    im1Reg = cv2.warpPerspective(im1, h, (width, height))
    
    return im1Reg

def find_reflection(image_0, image_1, threshold_value_min, threshold_value_max, min_area, max_area):
    '''
    This function finds reflections in two images by comparing them and identifying contours that meet specified area criteria.
    Input:
    image_0 : numpy.ndarray, the first image to compare
    image_1 : numpy.ndarray, the second image to compare
    threshold_value_min : int, minimum threshold value for binary thresholding
    threshold_value_max : int, maximum threshold value for binary thresholding
    min_area : int, minimum area of contours to be considered valid
    max_area : int, maximum area of contours to be considered valid
    Output:
    ((reflection_x, reflection_y), found, diff, thresh_with_contours, thresh_with_contours) : tuple
        reflection_x : int or None, x-coordinate of the centroid of the valid contour if found, otherwise None
        reflection_y : int or None, y-coordinate of the centroid of the valid contour if found, otherwise None
        found : bool, True if a valid contour is found, otherwise False
        diff : numpy.ndarray, the absolute difference between the two grayscale images
        thresh_with_contours : numpy.ndarray, the thresholded image with valid contours drawn on it
    '''
    # Allinea le due immagini
    #image_1_aligned = align_images(image_0, image_1)
    
    # Converte le immagini in scala di grigi
    gray_0 = cv2.cvtColor(image_0, cv2.COLOR_BGR2GRAY)
    gray_1 = cv2.cvtColor(image_1, cv2.COLOR_BGR2GRAY)

    # Calcola la differenza assoluta tra le immagini
    diff = cv2.absdiff(gray_0, gray_1)
    
    # Applica la threshold per ottenere un'immagine binaria
    _, thresh = cv2.threshold(diff, threshold_value_min, threshold_value_max, cv2.THRESH_BINARY)

    # Trova i contorni nell'immagine binaria
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Lista per contenere i contorni che soddisfano i criteri di area
    valid_contours = []
    
    for contour in contours:
        area = cv2.contourArea(contour)
        if min_area <= area <= max_area:
            valid_contours.append(contour)

    # Se ci sono contorni validi, calcola la posizione media
    reflection_x = None
    reflection_y = None
    found = False
    
    if len(valid_contours) > 0:
        found = True
        # Calcola la posizione media del centroide dei contorni validi
        moments = cv2.moments(valid_contours[0])
        reflection_x = int(moments['m10'] / moments['m00'])
        reflection_y = int(moments['m01'] / moments['m00'])
        
        print(f"Centroide medio del riflesso: X = {reflection_x} | Y = {reflection_y}")
    else:
        print("Nessun contorno valido trovato.")

        # Draw contours on the thresholded image
    thresh_with_contours = np.copy(thresh)  # Copy the thresh image to draw on
    cv2.drawContours(thresh_with_contours, valid_contours, -1, (155), 2)  # Draw contours in white (255) with thickness 2

    return ((reflection_x, reflection_y), found, diff, thresh_with_contours, thresh_with_contours)