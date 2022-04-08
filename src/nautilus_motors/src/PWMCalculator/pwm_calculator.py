import numpy as np
from geometry_msgs.msg import Wrench

# thruster order: ['forward_left', 'forward_right', 'forward_top', 'sideways_top', 'up_left', 'up_right']
# looks like a reasonable max on motor power is 3.87757526

class PWMCalculator:
    def __init__(self):
        self.transform_mat = np.array([[0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000012e+00,  0.00000000e+00,  0.00000000e+00],
                                       [-1.00000012e+00, -1.00000012e+00, -1.00000012e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00],
                                       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00,  1.00000000e+00],
                                       [-7.90000111e-02, -7.90000111e-02,  1.14700019e-01, -0.00000000e+00,  2.35999823e-02,  2.35999823e-02],
                                       [ 0.00000000e+00, -0.00000000e+00,  0.00000000e+00,  1.14700019e-01,  1.47949994e-01, -1.48149997e-01],
                                       [ 1.47950009e-01, -1.48150012e-01,  4.99950984e-05,  1.68400049e-01,  -0.00000000e+00,  0.00000000e+00]])

        self.transform_inv = np.linalg.inv(self.transform_mat)

        self.motor_scalars = [0.25, 0.25, 0.25, 0.25, 0.25, 0.25]
        self.max_power = 3.87757526 # max of a vertical motor going up and rolling, seems reasonable for now?

        def f(x):
            if x < 0:
                return int(np.interp(x, [-1, 0], [1100, 1475]))
            elif x > 0:
                return int(np.interp(x, [0, 1], [1525, 1900]))
            else:
                return 1500

        self.map_power_to_pwm = np.vectorize(f)

    def wrench_to_col(self, vector: Wrench) -> np.array:
        return np.array([[vector.force.x, vector.force.y, vector.force.z, vector.torque.x, vector.torque.y, vector.torque.z]]).T

    def convert_vector_to_pwms(self, vector: Wrench) -> list:
        # Calculate motor powers
        input_vector = self.wrench_to_col(vector)
        motor_powers = self.transform_inv @ input_vector

        # convert motor powers into pwms
        motor_powers = motor_powers / self.max_power # normalize the power vector
        return self.map_power_to_pwm(motor_powers).flatten().tolist()

