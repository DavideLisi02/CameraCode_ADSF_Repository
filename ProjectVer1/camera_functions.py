# camera_functions.py
import cv2
import numpy as np

class VideoCapture:
    def __init__(self, name):
        self.cap = cv2.VideoCapture(name)

    def read(self):
        success, frame = self.cap.read()
        return success, frame

def find_reflection(image_0, image_1, threshold_value_min, threshold_value_max, min_area, max_area):
    gray_0 = cv2.cvtColor(image_0, cv2.COLOR_BGR2GRAY)
    gray_1 = cv2.cvtColor(image_1, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(gray_0, gray_1)
    _, thresh = cv2.threshold(diff, threshold_value_min, threshold_value_max, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_contours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if min_area <= area <= max_area:
            valid_contours.append(contour)
    reflection_x = None
    reflection_y = None
    found = False
    if len(valid_contours) > 0:
        found = True
        moments = cv2.moments(valid_contours[0])
        reflection_x = int(moments['m10'] / moments['m00'])
        reflection_y = int(moments['m01'] / moments['m00'])
        print(f"Centroide medio del riflesso: X = {reflection_x} | Y = {reflection_y}")
    else:
        print("Nessun contorno valido trovato.")
    thresh_with_contours = np.copy(thresh)
    cv2.drawContours(thresh_with_contours, valid_contours, -1, (155), 2)
    return ((reflection_x, reflection_y), found, diff, thresh_with_contours, thresh_with_contours)


def align_images(im1, im2):
    # Currently not used
    MAX_FEATURES = 500
    GOOD_MATCH_PERCENT = 0.15
    im1Gray = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
    im2Gray = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create(MAX_FEATURES)
    keypoints1, descriptors1 = orb.detectAndCompute(im1Gray, None)
    keypoints2, descriptors2 = orb.detectAndCompute(im2Gray, None)
    matcher = cv2.DescriptorMatcher_create(cv2.DESCRIPTOR_MATCHER_BRUTEFORCE_HAMMING)
    matches = matcher.match(descriptors1, descriptors2)
    if isinstance(matches, tuple):
        matches = list(matches)
    matches.sort(key=lambda x: x.distance, reverse=False)
    numGoodMatches = int(len(matches) * GOOD_MATCH_PERCENT)
    matches = matches[:numGoodMatches]
    imMatches = cv2.drawMatches(im1, keypoints1, im2, keypoints2, matches, None)
    cv2.imwrite("matches.jpg", imMatches)
    points1 = np.zeros((len(matches), 2), dtype=np.float32)
    points2 = np.zeros((len(matches), 2), dtype=np.float32)
    for i, match in enumerate(matches):
        points1[i, :] = keypoints1[match.queryIdx].pt
        points2[i, :] = keypoints2[match.trainIdx].pt
    h, mask = cv2.findHomography(points1, points2, cv2.RANSAC)
    height, width, channels = im2.shape
    im1Reg = cv2.warpPerspective(im1, h, (width, height))
    return im1Reg

