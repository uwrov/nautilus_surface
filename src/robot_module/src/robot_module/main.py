import rospy
from .subscribers.image_sub import ImageSub
from .publishers.move_pub import MovePub
from .publishers.lock_pub import LockPub
from .subscribers.lock_sub import LockSub

# Service topics
publisher_topics = {
    "movement": lambda node_name: MovePub("/nautilus/motors/commands",
                                          node_name)
}
subscriber_topics = {
    "cameras": lambda: ImageSub("/nautilus/cameras/stream"),
    "img_h": lambda: ImageSub("/image/distribute")
}


class RobotModule:
    def __init__(self, name):
        rospy.init_node(name, log_level=rospy.DEBUG)
        self.active_services = {}
        self.name = name

    def setup(self, service):
        try:
            if service in publisher_topics:  # Service is a publisher
                self.active_services[service] = publisher_topics[service]()
            elif service in subscriber_topics:  # Service is a subscriber
                self.active_services[service] = subscriber_topics[service]()
            else:
                print("[ERROR]: Service not found")
        except Exception as e:
            print(e)

    def run_if_service(self, service, func):
        """
        Runs a function if the service is active
        :param service: Service name
        :param func: (lambda) Function to run
        """
        try:
            if service in self.active_services:
                func()
            else:
                print(f"[ERROR]: {service} not active yet")
        except Exception as e:
            print(e)

    def request_priority(self):
        self.run_if_service("movement",
                            lambda: self.active_services[
                                "movement"].request_priority()
                            )

    # Robot motor API
    def set_vel(self, linear, angular=[0, 0, 0]):
        """
        Sets the linear and angular velocity of the robot
        :param linear: [float, float, float] Linear velocity
        :param angular: [float, float, float] Angular velocity
        """
        self.run_if_service("movement",
                            lambda: self.active_services[
                                "movement"].set_vel(linear, angular)
                            )

    def sit(self, time):
        """
        Sits the robot for a certain amount of time
        :param time: [float] Time to sit
        """
        # TODO: Implement
        pass

    def stabilize(self):
        """
        Stabilize the ROV
        """
        # TODO: Implement
        pass

    def lock(self, axis):
        """
        Lock ROV movement to a specific axis
        :param axis: [string] Axis to lock
        """
        # TODO: Implement
        pass

    def kill_motors(self):
        """
        Kills the motors
        """
        # TODO: Implement
        pass

    def displace(self, vector):
        """
        Displace the ROV by a certain vector
        :param vector: [float, float, float] Displacement vector
        """
        # TODO: Implement
        pass

    def rotate(self, vector):
        """
        Rotate the ROV by a certain vector
        :param vector: [float, float, float] Rotation vector
        """
        # TODO: Implement
        pass

    def set_accel(self, vector):
        """
        Sets the acceleration of the ROV
        :param vector: [float, float, float] Acceleration vector
        """
        # TODO: Implement
        pass
