from robot_module import RobotModule

import random
import time

def go_forward(robot):
    for i in range(10):
        robot.set_vel([0, -0.5, 0], [0, 0, 0])
        time.sleep(2)
        robot.set_vel([0, 0, 0], [0, 0, 0])
        time.sleep(1)
        if i % 2 == 0:
            robot.set_vel([0, 0, 0], [0, 0, -0.05])
        else:
            robot.set_vel([0, 0, 0], [0, 0, 0.05])
        time.sleep(1)
        robot.set_vel([0, 0, 0], [0, 0, 0])


def main():
    """ Setup RobotModule """
    robot = RobotModule("surface")
    robot.setup("movement")

    go_forward(robot)

if __name__ == '__main__':
    main()
