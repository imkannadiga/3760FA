import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
import requests
import threading
import time
import os
from geometry_msgs.msg import TransformStamped
from dotenv import load_dotenv

UPLOAD_INTERVAL = 2  # seconds
DOWNLOAD_INTERVAL = 15  # seconds

class CostmapCloudSync(Node):
    def __init__(self):
        super().__init__('map_server_client')

        load_dotenv()

        self.MAP_SERVER_URL = os.getenv('MAP_SERVER')
        # Subscriptions
        self.subscription_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.subscription_global_costmap = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.global_costmap_callback, 10)
        self.subscription_local_costmap = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.local_costmap_callback, 10)
        self.subscription_tf = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)

        # Publishers
        # self.publisher_map = self.create_publisher(OccupancyGrid, '/map', 10)
        # self.publisher_global_costmap = self.create_publisher(OccupancyGrid, '/global_costmap/costmap', 10)
        # self.publisher_local_costmap = self.create_publisher(OccupancyGrid, '/local_costmap/costmap', 10)

        # Data Storage
        self.latest_map = None
        self.latest_global_costmap = None
        self.latest_local_costmap = None
        self.latest_transform = None

        self.lock = threading.Lock()

        # Threads
        self.upload_thread = threading.Thread(target=self.upload_loop, daemon=True)
        # self.download_thread = threading.Thread(target=self.download_loop, daemon=True)
        self.upload_thread.start()
        # self.download_thread.start()

    # ===================== CALLBACKS =====================
    def map_callback(self, msg):
        with self.lock:
            self.latest_map = msg

    def global_costmap_callback(self, msg):
        with self.lock:
            self.latest_global_costmap = msg

    def local_costmap_callback(self, msg):
        with self.lock:
            self.latest_local_costmap = msg

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == "base_footprint":  # Assuming robot frame is "base_link"
                with self.lock:
                    self.latest_transform = transform

    # ===================== UPLOAD LOOP =====================
    def upload_loop(self):
        while rclpy.ok():
            # time.sleep(UPLOAD_INTERVAL)
            # with self.lock:
                if self.latest_map and self.latest_global_costmap and self.latest_local_costmap and self.latest_transform:
                    self.upload_data()

    def upload_data(self):
        try:
            data = {
                "map": self.serialize_map(self.latest_map),
                "global_costmap": self.serialize_map(self.latest_global_costmap),
                "local_costmap": self.serialize_map(self.latest_local_costmap),
                "transform": self.serialize_transform(self.latest_transform)
            }
            response = requests.post(f"{self.MAP_SERVER_URL}/api/costmap/upload_map", json=data)
            response.raise_for_status()
            self.get_logger().info("Maps and transform uploaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to upload data: {e}")

    # ===================== DOWNLOAD LOOP =====================
    def download_loop(self):
        while rclpy.ok():
            time.sleep(DOWNLOAD_INTERVAL)
            self.download_data()

    def download_data(self):
        self.get_logger().info("Downloading latest maps and transform...")
        try:
            response = requests.get(f"{self.MAP_SERVER_URL}/api/costmap/download_map")
            response.raise_for_status()
            data = response.json()

            with self.lock:
                # Publish the downloaded maps
                self.publisher_map.publish(self.deserialize_map(data["map"]))
                self.publisher_global_costmap.publish(self.deserialize_map(data["global_costmap"]))
                self.publisher_local_costmap.publish(self.deserialize_map(data["local_costmap"]))
            
            self.get_logger().info("Maps and transform updated.")
        except Exception as e:
            self.get_logger().error(f"Failed to download data: {e}")

    # ===================== HELPER FUNCTIONS =====================
    def serialize_map(self, msg: OccupancyGrid):
        return {
            "info": {
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": msg.info.resolution,
                "origin": {
                    "x": msg.info.origin.position.x,
                    "y": msg.info.origin.position.y,
                    "z": msg.info.origin.position.z,
                }
            },
            "header": {"stamp": str(msg.header.stamp.sec)},
            "data": list(msg.data)
        }

    def serialize_transform(self, transform: TransformStamped):
        return {
            "translation": {
                "x": transform.transform.translation.x,
                "y": transform.transform.translation.y,
                "z": transform.transform.translation.z
            },
            "rotation": {
                "x": transform.transform.rotation.x,
                "y": transform.transform.rotation.y,
                "z": transform.transform.rotation.z,
                "w": transform.transform.rotation.w
            }
        }

    def deserialize_map(self, data):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.width = data["width"]
        msg.info.height = data["height"]
        msg.info.resolution = data["resolution"]
        msg.data = data["data"]
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = CostmapCloudSync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
