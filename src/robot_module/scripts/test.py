from robot_module import RobotModule
import time

robot = RobotModule("test")

robot.setup("movement")

robot.set_vel([0, 0, 0], [0, 0, 1])

time.sleep(15) # Sleep for 3 seconds
