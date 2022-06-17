from robot_module import RobotModule

import random
import time

robot = None


def go_forward():
    for i in range(10):
        robot.set_vel([0, -0.1, 0], [0, 0, 0])
        time.sleep(2)
        robot.set_vel([0, 0, 0], [0, 0, 0])
        time.sleep(1)
        if i % 2 == 0:
            robot.set_vel([0, 0, 0], [0, 0, -0.01])
        else:
            robot.set_vel([0, 0, 0], [0, 0, 0.01])
        time.sleep(1)
        robot.set_vel([0, 0, 0], [0, 0, 0])


def main():
    """ Setup RobotModule """
    robot = RobotModule("surface")
        robot.setup("movement")

if __name__ == '__main__':
    main()
