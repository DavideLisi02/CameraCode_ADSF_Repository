import cv2
import os
import numpy as np
import requests
import time
import queue
import threading
import cvlib as cv
import matplotlib.pyplot as plt
from cvlib.object_detection import draw_bbox

class VideoCapture:
 
  def __init__(self, name):
    self.cap = cv2.VideoCapture(name)
    self.q = Queue.Queue()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()
    
  def _reader(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()
        except Queue.Empty:
          pass
      self.q.put(frame)
 
  def read(self):
    return self.q.get()    

def findObjinImage(room_image, object_image):
    room_gray = cv2.cvtColor(room_image, cv2.COLOR_BGR2GRAY)
    plant_gray = cv2.cvtColor(object_image, cv2.COLOR_BGR2GRAY)

    # Perform template matching
    result = cv2.matchTemplate(room_gray, plant_gray, cv2.TM_CCOEFF_NORMED)

    # Define a threshold to find the best matches
    threshold = 0.98  # Adjust based on similarity; closer to 1 means stricter matching

    # Get the coordinates of matching regions
    locations = np.where(result >= threshold)

    # If you want to get the single best match location:
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

    # `max_loc` is the top-left corner of the plant location in the room image
    plant_top_left = max_loc
    plant_bottom_right = (plant_top_left[0] + 10, plant_top_left[1] + 10)

    # Draw a rectangle around the detected plant in the room image (optional, for visualization)
    cv2.rectangle(room_image, plant_top_left, plant_bottom_right, (0, 255, 0), 2)
    return result
 
os.chdir(os.path.dirname(os.path.abspath(__file__)))
URL = "http://192.168.121.87"
cap = cv2.VideoCapture(URL + ":81/stream")
object_image = cv2.imread('Mouse1.jpg')
cv2.imshow('Mouseimg',object_image)

if __name__ == '__main__':
    requests.get(URL + "/control?var=framesize&val={}".format(8))
    while True:
        if cap.isOpened():
            ret, frame = cap.read()
            
            findObjinImage(frame, object_image)

            cv2.imshow("Output", frame)
 
            #imgnp=np.array(bytearray(cap.read()),dtype=np.uint8)
            #im = cv2.imdecode(imgnp,-1)
            #bbox, label, conf = cv.detect_common_objects(frame)
            #im = draw_bbox(frame, bbox, label, conf)
            #cv2.imshow('Output',im)
 
            key = cv2.waitKey(3)
            
            if key == 27:
                break
 
    cv2.destroyAllWindows()
    cap.release()