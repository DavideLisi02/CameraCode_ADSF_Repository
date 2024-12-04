import time
import serial
import os
import cv2
import numpy as np


os.system('cls')  # Clear the terminal screen

def add_line_to_file(file_path, line):
    """Adds a line to a text file.

    Args:
        file_path: The path to the text file.
        line: The string to add to the file.
    """
    try:
        with open(file_path, 'a') as file:
            file.write(line + '\n')
    except FileNotFoundError:
        print(f"Error: File not found at {file_path}")
    except Exception as e:
        print(f"An error occurred: {e}")

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
        biggest_contour = None
        biggest_area = 0
        for contour in valid_contours:
            if cv2.contourArea(contour) > biggest_area:
                biggest_area = cv2.contourArea(contour)
                biggest_contour = contour
        print(f"Biggest contour area: {biggest_area}")
        print(f"Biggest contour: {biggest_contour}")
        moments = cv2.moments(biggest_contour)

        if moments['m00'] != 0: #Avoid division by zero
            reflection_x = int(moments['m10'] / moments['m00'])
            reflection_y = int(moments['m01'] / moments['m00'])
        
        reflection_x = int(reflection_x )
        reflection_y = int(reflection_y)
        print(f"Average centroid of reflection: X = {reflection_x} | Y = {reflection_y}")
    else:
        print("No valid contours found.")
    thresh_with_contours = np.copy(thresh)
    cv2.drawContours(thresh_with_contours, valid_contours, -1, (155), 2)
    print(found)
    return ((reflection_x, reflection_y), found, diff, thresh_with_contours, thresh_with_contours)



esp32 = serial.Serial(port='COM9', baudrate=512000, timeout=1)
time.sleep(2)
 
##########################################################
#URL
URL = 0

# Global variables for threshold values
area_value_min = 1000
area_value_max = 500000
threshold_value_min = 40
threshold_value_max = 255

#! Settings for control loop

# Com port for serial comunication with the esp32
com_port = 'COM9'

# Constraints for motor movement
UP_constr_1 = 180
LOW_constr_1 = 0
UP_constr_2 = 180
LOW_constr_2 = 0

# No reflection position of the motors
no_ref_motor1 = 90
no_ref_motor2 = 93

# Reflection ensured position of the motors
yes_ref_motor1 = 90
yes_ref_motor2 = 85

def calculate_control(x_p,y_p,x_r,y_r):
    #i_1 = pid_control(Kp_1, Ki_1, Kd_1, x_p, x_r, (LOW_constr_1,UP_constr_1))
    #i_2 = pid_control(Kp_2, Ki_2, Kd_2, y_p, y_r, (LOW_constr_2,UP_constr_2))
    i_2 = int(0.01*(x_p-x_r)+90)
    i_1 = int(0.01*(y_r-y_p)+90)

    if i_1>LOW_constr_1 and i_2>LOW_constr_2:
        i_1 = min(i_1,UP_constr_1)
        i_2 = min(i_2,UP_constr_2)
    elif i_1>LOW_constr_1 and i_2<LOW_constr_2:
        i_1 = min(i_1,UP_constr_1)
        i_2 = LOW_constr_2
    elif i_1<LOW_constr_1 and i_2>LOW_constr_2:
        i_1 = LOW_constr_1
        i_2 = min(i_2,UP_constr_2)
    else:
        i_1 = LOW_constr_1
        i_2 = LOW_constr_2
    return [i_1,i_2]

def move(i_1, i_2, esp32):
    print(f"##########################################################\nTrying to move the motors")
    msgWR = f"motor1:{int(i_1)} motor2:{int(i_2)}\n"
    print(f"Sending: {msgWR}")
    esp32.write(bytes(msgWR, 'utf-8'))
    add_line_to_file("controls.txt", msgWR)

    
'''
def motor_loop():
    global i_1
    global i_2
    global esp32
    while True:
        move(i_1,i_2,esp32)
        print("#####################################################################")
'''
# Initial movements
move(no_ref_motor1, no_ref_motor2, esp32)
time.sleep(0.5)
move(90, 90, esp32)
time.sleep(1)


print("Starting video stream...")
cap = cv2.VideoCapture(URL)
success, initial_frame = cap.read()

tracker = cv2.TrackerKCF_create()
bbox_initialized = False
success, frame = cap.read()
frame_copy = frame.copy()


time.sleep(0.5)
move(yes_ref_motor1, yes_ref_motor2, esp32)
time.sleep(0.3)
move(90, 90, esp32)
print("moved back")
i_1 = 90
i_2 = 90

x_p = 0
y_p = 0

#motor_loop_thread = threading.Thread(target=motor_loop)
#motor_loop_thread.start()

#Main loop
while True:
    success, frame = cap.read()

    frame_copy = frame.copy()
    
    if not bbox_initialized:
        bbox = cv2.selectROI("Video Stream", initial_frame, False)
        tracker.init(frame, bbox)
        bbox_initialized = True

    success, bbox = tracker.update(frame)



    if success:
        (x_b, y_b, w, h) = [int(v) for v in bbox]
        cv2.rectangle(frame, (x_b, y_b), (x_b + w, y_b + h), (0, 255, 0), 2, 1)
        print(f"Object detected at coord = X:{(x_b + w) / 2} | Y:{(y_b + h) / 2}")
        x_p = x_b+w/2
        y_p = y_b+h/2
        while x_p > x_p*1.1 or x_p < x_p*0.9 or y_p > y_p*1.1 or y_p < y_p*0.9:
            success, frame = cap.read()
            success, bbox = tracker.update(frame)
            if success:
                (x_b, y_b, w, h) = [int(v) for v in bbox]
                cv2.rectangle(frame, (x_b, y_b), (x_b + w, y_b + h), (0, 255, 0), 2, 1)
                print(f"Object detected at coord = X:{(x_b + w) / 2} | Y:{(y_b + h) / 2}")
                x_p = x_b+w/2
                y_p = y_b+h/2
    else:
        cv2.putText(frame, "Tracking failure", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    reflection_xy = find_reflection(initial_frame, frame_copy, threshold_value_min, threshold_value_max, area_value_min, area_value_max)        

    print(f"###################\nReflection: {reflection_xy[1]}\n##########################")

    if reflection_xy[1]:
        
        cv2.circle(frame, reflection_xy[0], 10, (0, 0, 255), 2)
        
        x_r = reflection_xy[0][0]
        y_r = reflection_xy[0][1]

        [i_1,i_2] = calculate_control(x_p, y_p, x_r, y_r)      
        move(i_1, i_2, esp32)      
         #move(90,90,esp32)
        # Draw an arrow from (x_r, y_r) to (x_r + i_1, y_r + i_2)
        cv2.arrowedLine(frame, (x_r, y_r), (x_r + i_1 -90, y_r + i_2-90), (255, 0, 0), 2)
    else:
        move(90,90,esp32)

    gray_diff = cv2.resize(reflection_xy[2], (400, 400))
    threshold = cv2.resize(reflection_xy[3], (400, 400))

    cv2.arrowedLine(frame, (0, 0), (40, 0), (255, 0, 0), 2)  # x direction
    cv2.arrowedLine(frame, (0, 0), (0, 40), (255, 0, 0), 2)  # y direction
    cv2.putText(frame, 'x', (45, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA) # x label
    cv2.putText(frame, 'y', (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA) # y label

    cv2.imshow("Gray scale difference", gray_diff)
    cv2.imshow("Threshold", threshold)
    cv2.imshow("Video Stream", frame)
    cv2.moveWindow("Gray scale difference", 0, 0)# 0 0
    cv2.moveWindow("Threshold", 420, 0) # 420 0
    cv2.moveWindow("Video Stream", 840, 0) # 0 420
    '''UNCIMMENT THIS IF YOU HAVE A SECOND SCREEN
    cv2.moveWindow("Video Stream", 1920, 0)  # Move to the second screen
    cv2.setWindowProperty("Video Stream", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)  # Set to fullscreen
    '''
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
move(90, 90, esp32)
esp32.close()
