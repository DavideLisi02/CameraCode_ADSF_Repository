from main import A_1
from main import A_2
from main import UP_constr_1
from main import LOW_constr_1
from main import UP_constr_2
from main import LOW_constr_2


def move_motors(i1,i2,esp32):
    '''
    This function send values to a microcontroller by serial port.
    
    Input:
    i1 : integer value, value to control motor1
    i2 : integer value, value to control motor2
    esp32 : object, variable related to the microcontroller in use
    
    Output:
    [i1,i2] : list
    '''

    msgWR = f"motor1:{i1} motor2:{i2}"
    print(f"THIS INPUT WAS GIVEN TO THE MOTORS:\nMotor1: {i1}\nMotor2: {i2}")
    print(f"Sending message to motors: {msgWR}")  # Debug: Print the message being sent to ESP32
    esp32.write(bytes(msgWR, 'utf-8'))  # Write the input message to the ESP32
    return [i1, i2]


def control_motors(dx,dy,i_01,i_02,esp32):
    '''
    This function calculates and sends values to a microcontroller via serial port according to
    the definition of the system used for the control loop.
    
    Input:
    dx: integer value, plant_x - reflection_x
    dy: integer value, plant_y - reflection_y
    i_01: integer value, last value used to control motor1 (last position of motor1)
    i_02: integer value, last value used to control motor2 (last position of motor2)
    esp32: object, variable related to the microcontroller in use
    
    Output:
    [i_11, i_12]: list, values calculated and used to control motors 1 and 2 respectively according to
    the system [i_11, i_12] = [A_1 0, 0 A_2] * [dx, dy]
    '''
    
    di_1 = dx * A_1
    di_2 = dy * A_2
    i_11 = int(i_01 + di_1)
    i_12 = int(i_02 + di_2)
    if i_11>LOW_constr_1 and i_12>LOW_constr_2:
        i_11 = min((i_11,UP_constr_1))
        i_12 = min(i_12,UP_constr_2)
    elif i_11>LOW_constr_1 and i_12<LOW_constr_2:
        i_11 = min((i_11,UP_constr_1))
        i_12 = LOW_constr_2
    else:
        i_11 = LOW_constr_1
        i_12 = min(i_12,UP_constr_2)
        
    msgWR = f"motor1:{i_11} motor2:{i_12}"
    print(f"THIS INPUT WAS GIVEN TO THE MOTORS:\nMotor1: {i_11}\nMotor2: {i_12}")
    print(f"Sending message to motors: {msgWR}")  # Debug: Print the message being sent to ESP32
    esp32.write(bytes(msgWR, 'utf-8'))  # Write the input message to the ESP32
    return [i_11,i_12]
