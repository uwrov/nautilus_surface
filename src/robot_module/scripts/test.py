from robot_module import RobotModule
import time

robot = RobotModule("test")

robot.setup("movement")
robot.request_priority()

for i in range(10):
    print("sending test")
    robot.set_vel([0, 0, 0], [0, 0, 1])
    time.sleep(1) # Sleep for 3 seconds
