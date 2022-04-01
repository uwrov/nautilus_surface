import rospy
from subscribers.image_sub import ImageSub
from publishers.move_pub import MovePub

# Service topics
publisher_topics = {
    "movement": MovePub("/nautilus/motors/commands"),
}
subscriber_topics = {
    "cameras": ImageSub("/nautilus/cameras/stream"),
    "img_h": ImageSub("/image/distribute")
}


class RobotModule:
    def __init__(self):
        self.active_services = {}

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

    # Robot motor API
    def set_vel(self, vector):
        try:
            if "movement" in self.active_services:
                self.active_services["movement"].update_state(vector)
            else:
                print("[ERROR]: Movement not active yet")
        except Exception as e:
            print(e)

    def sit(self, time):
        try:
            if "movement" in self.active_services:
                # TODO: Implement
                pass
            else:
                print("[ERROR]: Movement not active yet")
        except Exception as e:
            print(e)

    def stabilize(self):
        try:
            if "movement" in self.active_services:
                # TODO: Implement
                pass
            else:
                print("[ERROR]: Movement not active yet")
        except Exception as e:
            print(e)

    def lock(self, axis):
        try:
            if "movement" in self.active_services:
                # TODO: Implement
                pass
            else:
                print("[ERROR]: Movement not active yet")
        except Exception as e:
            print(e)

    def kill_motors(self):
        try:
            if "movement" in self.active_services:
                # TODO: Implement
                pass
            else:
                print("[ERROR]: Movement not active yet")
        except Exception as e:
            print(e)

    def displace(self, vector):
        try:
            if "movement" in self.active_services:
                # TODO: Implement
                pass
            else:
                print("[ERROR]: Movement not active yet")
        except Exception as e:
            print(e)

    def rotate(self, vector):
        try:
            if "movement" in self.active_services:
                # TODO: Implement
                pass
            else:
                print("[ERROR]: Movement not active yet")
        except Exception as e:
            print(e)

    def set_accel(self, vector):
        try:
            if "movement" in self.active_services:
                # TODO: Implement
                pass
            else:
                print("[ERROR]: Movement not active yet")
        except Exception as e:
            print(e)

    def start_api(self):
        try:
            if "movement" in self.active_services:
                # TODO: Implement
                pass
            else:
                print("[ERROR]: Movement not active yet")
        except Exception as e:
            print(e)

    def stop_api(self):
        try:
            if "movement" in self.active_services:
                # TODO: Implement
                pass
            else:
                print("[ERROR]: Movement not active yet")
        except Exception as e:
            print(e)