import carla
from carla_msgs.msg import CarlaWorldInfo
from nav_msgs.msg import Path

import ros_compatibility as roscomp
from ros_compatibility.exceptions import *
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy


class WayPointSubscriber(CompatibleNode):
    def __init__(self):
        super().__init__('waypoint_subscriber')
        self.connect_to_carla()
        self.goal_subscriber = self.create_subscription(Path, "/carla/ego_vehicle/waypoints", self.waypoint_visualizer, qos_profile=10)

    def waypoint_visualizer(self, msg):
        self.loginfo('Waypoints: {}'.format(msg.poses[0].pose.position.x))
        for waypoint_pose in msg.poses:
            # y must be negated for conversion from ros2 env to CARLA
            waypoint_position = carla.Location(x = waypoint_pose.pose.position.x, y = -waypoint_pose.pose.position.y, z = waypoint_pose.pose.position.z)
            self.world.debug.draw_string(waypoint_position, '^', draw_shadow=False,
                            color=carla.Color(r=0, g=0, b=255), life_time=10.0, persistent_lines=True)


    def connect_to_carla(self):
        self.loginfo("Waiting for CARLA world (topic: /carla/world_info)...")
        try:
            self.wait_for_message(
                "/carla/world_info",
                CarlaWorldInfo,
                qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                timeout=15.0)
        except ROSException as e:
            self.logerr("Error while waiting for world info: {}".format(e))
            raise e

        host = self.get_param("host", "127.0.0.1")
        port = self.get_param("port", 2000)
        timeout = self.get_param("timeout", 10)
        self.loginfo("CARLA world available. Trying to connect to {host}:{port}".format(
            host=host, port=port))

        carla_client = carla.Client(host=host, port=port)
        carla_client.set_timeout(timeout)

        try:
            self.world = carla_client.get_world()
        except RuntimeError as e:
            self.logerr("Error while connecting to Carla: {}".format(e))
            raise e

        self.loginfo("Connected to Carla.")

def main(args=None):
    roscomp.init('waypoint_subscriber', args)
    waypoint_subscriber = None
    try:
        waypoint_subscriber = WayPointSubscriber()
        waypoint_subscriber.spin()
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    finally:
        roscomp.loginfo("Shutting down.")
        if waypoint_subscriber:
            waypoint_subscriber.destroy()
        roscomp.shutdown()