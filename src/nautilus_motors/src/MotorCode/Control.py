import time
import numpy as np
from geometry_msgs.msg import Wrench

# Placeholder values giving a range for use in roation.
min = 500
max = 2500
slope = (max - min) / 2
intercept = max - slope * (1)


inputs = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 0]]

# Gives the default values for the pins.
pin1 = 21 
pin2 = 20 
pin3 = 16

#pin4 = 12 #original pin
pin4 = 19
pin5 = 26

#pin6 = 19 # original pin
pin6 = 12

# scalers applied onto each motors, index 0 is motor A and index 5 is motor F
motor_scalers = [0.25, -0.25, -0.25, 0.25, 0.25, 0.25]


# controlOutputs - array of pwms to set
# idx - index of the array to set (0 -> A, 5->F)
# c - motor constant, value in [-1,1] which scales output of motor, 
#     0 is stop, -1 is full blast reverse, 1 is full blast forward
def pwm_transform(controlOutputs, idx, c):
    global motor_scalers
    controlOutputs[idx] = (motor_scalers[idx] * slope * c) + intercept 

# Takes in a wrench message describing desired motion of the ROV and 
# returns 6 floats representing the pwms to be applied onto each motor.
def calculate_pwms(controlInputs: Wrench) -> list:
    controlOutputs = [0, 0, 0, 0, 0, 0]

    # convert wrench to regular arry
    controlInputs = [controlInputs.force.x, controlInputs.force.y, controlInputs.force.z,
                     controlInputs.torque.x, controlInputs.torque.y, controlInputs.torque.z]

    # Checks if the given input array is valid and shows an error message if not.
    for item in controlInputs:
        assert item >= -1 or item <= 1

    # flip inputs as needed
    controlInputs[0] *= -1      # X 
    controlInputs[1] *= 1      # Y
    controlInputs[2] *= -1      # Z
    controlInputs[3] *= 1      # R

    # Case where there is no rotation based on the "R" value of the input array being zero.
    if (controlInputs[3] == 0):
        # calculate motor constants
        Ac = 0.5 * (controlInputs[0] - controlInputs[1])
        Bc = 0.5 * (controlInputs[0] + controlInputs[1])
        Cc = 0.5 * (controlInputs[0] + controlInputs[1])
        Dc = 0.5 * (controlInputs[0] - controlInputs[1])
        Ec = controlInputs[2]
        Fc = Ec
        
        # apply constants onto motors
        pwm_transform(controlOutputs, 0, Ac) 
        pwm_transform(controlOutputs, 1, Bc) 
        pwm_transform(controlOutputs, 2, Cc) 
        pwm_transform(controlOutputs, 3, Dc) 
        pwm_transform(controlOutputs, 4, Ec) 
        pwm_transform(controlOutputs, 5, Fc) 
    else:
        # rotation is clockwise based on a positive "R" value.
        c1 = controlInputs[3] / 2
        c2 = -1 * c1
        
        pwm_transform(controlOutputs, 0, c2)
        pwm_transform(controlOutputs, 1, c1)
        pwm_transform(controlOutputs, 2, c2)
        pwm_transform(controlOutputs, 3, c1)

        # zero out vertical motors
        pwm_transform(controlOutputs, 4, 0)
        pwm_transform(controlOutputs, 5, 0)
    return controlOutputs

if __name__ == '__main__':
    
    outputs = [1500, 1500, 1500, 1500, 1500, 1500]
    #outputs = control(inputs)
    # Operates the servo connected to each pin at a specific power.
    print(outputs)
    time.sleep(20)

# Test Code Runthrough:
# [0, 0, 0, 0]
# Connects with the raspberry pi device. [pi = pigpio.pi()]
# # Sets the signal pin value on the raspberry pi of each motor.
# [pin1 = 16; pin2 = 12; pin3 = 26;  pin4 = 19; pin5 = 21;  pin6 = 20]
# output = [0, 0, 0, 0, 0, 0]
# Process the control values for each motor.
