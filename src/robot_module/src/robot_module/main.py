import rospy
from .services.movement import Movement
from .services.images import RobotImages
from .services.manipulator import Manipulator

# Service topics
# subscriber_topics = {
#    "cameras": lambda: ImageSub("/nautilus/cameras/stream"),
#    "img_h": lambda: ImageSub("/image/distribute")
#}

# Available Services
services = {
    "movement": lambda node_name: Movement(node_name),
    "manipulator": lambda node_name: Manipulator(node_name),
    "images": lambda node_name: RobotImages(node_name)
}

class RobotModule:
    def __init__(self, name):
        rospy.init_node(name, log_level=rospy.DEBUG)
        self.active_services = {}
        self.name = name


    def setup(self, service):
        try:
            if service in services:  # Service is a publisher
                self.active_services[service] = services[service](self.name)
                self.active_services[service].setup()
            else:
                print("[ERROR]: Service not found")
        except Exception as e:
            print(e)


    def __run_if_service(self, service, func):
        """
        Runs a function if the service is active
        :param service: Service name
        :param func: (lambda) Function to run
        """
        try:
            if service in self.active_services:
                 return func()
            else:
                print(f"[ERROR]: {service} not active yet")
        except Exception as e:
            print(e)


    def arm_motors(self):
        self.__run_if_service("movement",
                            lambda: self.active_services[
                                "movement"].arm()
                            )


    def request_priority(self):
        self.__run_if_service("movement",
                            lambda: self.active_services[
                                "movement"].request_priority()
                            )


    def set_angle(self, percentage):
        """
        Sets the percentage of how open the manipulator is open
        100 being maximum and 0 being minimum
        :param percentage: int percentage of manipulator being open
        """
        self.__run_if_service("manipulator",
                            lambda: self.active_services["manipulator"]
                                .set_angle(percentage)
                            )


    # Robot motor API
    def set_vel(self, linear, angular=[0, 0, 0]):
        """
        Sets the linear and angular velocity of the robot
        :param linear: [float, float, float] Linear velocity
        :param angular: [float, float, float] Angular velocity
        """
        self.__run_if_service("movement",
                            lambda: self.active_services[
                                "movement"].set_velocity(linear, angular)
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

    def get_image_topics(self):
        return self.__run_if_service("images",
                    lambda: self.active_services["images"]
                        .get_image_topics()
                    )

    def get_image(self, topic):
        """
        Returns the current camera image available from the robot.
        :param int: index of the image
        """
        return self.__run_if_service("images",
                            lambda: self.active_services["images"]
                                .get_image(topic)
                            )


    def add_image_listener(self, listener):
        """
        add a callback function for the image stream. Listener is called on update
        of the image.
        """
        self.__run_if_service("images",
                            lambda: self.active_services["images"]
                                .callbacks.append(listener)
                            )
