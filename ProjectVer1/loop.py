import camera_functions
import controller_functions
import settings
import cv2
import threading
import queue

# Event to signal thread termination
stop_event = threading.Event()

def start_video_stream_fun(URL, max_features, good_match_percent, threshold_min, threshold_max, area_min, area_max, q):
    print("Starting video stream...")
    restart_loop = False
    while not stop_event.is_set():
        try:
            cap = cv2.VideoCapture(URL)
            print(f"Attempting to open video source: {URL}")
            if not cap.isOpened():
                print(f"Could not open video source: {URL}")
                q.put({"error": f"Could not open video source: {URL}"})
                break

            success, initial_frame = cap.read()
            print(f"Attempting to read initial frame: success={success}")
            if not success:
                print("Could not read frame from video source.")
                q.put({"error": "Could not read frame from video source."})
                break

            tracker = cv2.TrackerKCF_create()
            bbox_initialized = False
            while not stop_event.is_set() and not restart_loop:
                print("Inside main loop...")
                success, frame = cap.read()
                if not success:
                    break
                frame_copy = frame.copy()
                if not bbox_initialized:
                    bbox = cv2.selectROI("Video Stream", frame, False)
                    try:
                        tracker.init(frame, bbox)
                        bbox_initialized = True
                    except Exception as e:
                        print(f"Tracker initialization failed: {e}")
                        q.put({"error": f"Tracker initialization failed: {e}"})
                        break

                success, bbox = tracker.update(frame)
                if success:
                    (x, y, w, h) = [int(v) for v in bbox]
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2, 1)
                    print(f"Object detected at coord = X:{(x + w) / 2} | Y:{(y + h) / 2}")
                else:
                    cv2.putText(frame, "Tracking failure", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
                reflection_xy = camera_functions.find_reflection(initial_frame, frame_copy, threshold_min, threshold_max, area_min, area_max)
                if reflection_xy[1]:
                    cv2.circle(frame, reflection_xy[0], 10, (0, 0, 255), 2)
                resized_gray_diff = cv2.resize(reflection_xy[2], (400, 400))
                resized_threshold = cv2.resize(reflection_xy[3], (400, 400))
                resized_frame = cv2.resize(frame, (400, 400))
                q.put({"frame": resized_frame, "gray_diff": resized_gray_diff, "threshold": resized_threshold})
                cv2.imshow("Gray scale difference", resized_gray_diff)
                cv2.imshow("Threshold", resized_threshold)
                cv2.imshow("Video Stream", resized_frame)
                cv2.moveWindow("Gray scale difference", 0, 0)
                cv2.moveWindow("Threshold", 420, 0)
                cv2.moveWindow("Video Stream", 840, 0)
                if cv2.waitKey(3) & 0xFF == ord('r'):
                    restart_loop = True
                    break
                if cv2.waitKey(3) & 0xFF == ord('q'):
                    stop_event.set()
                    break
            restart_loop = False
            bbox_initialized = False

            cap.release()
            cv2.destroyAllWindows()
        except Exception as e:
            print(f"An error occurred: {e}")
            q.put({"error": f"An error occurred: {e}"})
            break

    print("Video stream stopped.")
    q.put({"done": True})
