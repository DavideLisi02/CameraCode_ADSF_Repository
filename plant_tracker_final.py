import cv2
import os
import numpy as np
import requests
import queue
import threading
import cvlib as cv
from cvlib.object_detection import draw_bbox

class VideoCapture:
 
  def __init__(self, name):
    self.cap = cv2.VideoCapture(name)
    self.q = queue.Queue()
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
        except queue.Empty:
          pass
      self.q.put(frame)
 
  def read(self):
    return self.q.get()    

def findObjinImage(room_image, object_image):
    room_gray = cv2.cvtColor(room_image, cv2.COLOR_BGR2GRAY)
    plant_gray = cv2.cvtColor(object_image, cv2.COLOR_BGR2GRAY)
    result = cv2.matchTemplate(room_gray, plant_gray, cv2.TM_CCOEFF_NORMED)
    threshold = 0.98
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    plant_top_left = max_loc
    plant_bottom_right = (plant_top_left[0] + 10, plant_top_left[1] + 10)
    cv2.rectangle(room_image, plant_top_left, plant_bottom_right, (0, 255, 0), 2)
    return result

os.chdir(os.path.dirname(os.path.abspath(__file__)))
URL = "http://192.168.137.129"
tracker = cv2.TrackerKCF_create()
cap = cv2.VideoCapture(URL + ":81/stream")

# Define video writer parameters
output_file = 'output_video.avi'
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = 20.0  # Set frames per second
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Define codec
out = cv2.VideoWriter(output_file, fourcc, fps, (frame_width, frame_height))

first_iteration = True
if __name__ == '__main__':
    requests.get(URL + "/control?var=framesize&val={}".format(8))
    while True:
        if cap.isOpened():
            ret, frame = cap.read()
            
            if ret and first_iteration:
                init_box = cv2.selectROI(frame, False)
                tracker.init(frame, init_box)
                first_iteration = False

            if not ret:
               break

            ret, bbox = tracker.update(frame)
            if ret:
                (x, y, w, h) = [int(v) for v in bbox]
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2, 1)
                print(f"Object detected at coord = X:{(x+w)/2} | Y:{(y+h)/2}")
            else:
                cv2.putText(frame, "Tracking failure", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

            # Write the frame to the video file
            out.write(frame)
            
            cv2.imshow("Output", frame)
            key = cv2.waitKey(3)
            
            if key == 27:  # Press 'Esc' to exit
                break

    # Release resources
    cap.release()
    out.release()
    cv2.destroyAllWindows()

