import rospy


class SubInfo:
    def __init__(self, ros_topic, sio_route, sio_id, sub):
        self.ros_topic = ros_topic
        self.sio_route = sio_route
        self.sio_id = sio_id
        self.sub = sub


class PubInfo:
    def __init__(self, ros_topic, pub):
        self.ros_topic = ros_topic
        self.pub = pub


class RobotModule:
    def __init__(self):
        # TODO: [Needs to be class constants] List of publishers and subscribers
        self.publishers = {
            "move_h": PubInfo("/nautilus/motors/commands", None),
        }
        self.subscribers = {
            "camera_h": SubInfo(
                "/nautilus/cameras/stream", "Image Display", "camera_stream", None
            ),
            "img_h": SubInfo("/image/distribute", "Image Display", "img_sub", None),
        }

        # TODO: List of activated publishers and subscribers
        self.active_publishers = []
        self.active_subscribers = []
    
    def setup(node):
        # TODO: Given the name of a node, assign publisher or subscriber to the node and append to the active list
        pass
