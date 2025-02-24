import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import requests
import threading
import time

CLOUD_SERVER_URL = "http://localhost:10002/api/costmap"
UPLOAD_INTERVAL = 5  # seconds
DOWNLOAD_INTERVAL = 15  # seconds

class CostmapCloudSync(Node):
    def __init__(self):
        super().__init__('map_server_client')
        
        self.subscription_global = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            10
        )
        self.subscription_local = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_callback,
            10
        )
        
        self.publisher = self.create_publisher(OccupancyGrid, '/global_costmap/costmap', 10)
        
        self.latest_costmap = None
        self.lock = threading.Lock()
        
        self.upload_thread = threading.Thread(target=self.upload_loop, daemon=True)
        self.download_thread = threading.Thread(target=self.download_loop, daemon=True)
        self.upload_thread.start()
        self.download_thread.start()
    
    def costmap_callback(self, msg):
        with self.lock:
            self.latest_costmap = msg
    
    def upload_loop(self):
        while rclpy.ok():
            time.sleep(UPLOAD_INTERVAL)
            with self.lock:
                if self.latest_costmap:
                    self.upload_costmap(self.latest_costmap)
    
    def download_loop(self):
        while rclpy.ok():
            time.sleep(DOWNLOAD_INTERVAL)
            self.download_costmap()
    
    def upload_costmap(self, costmap):
        self.get_logger().info("Uploading costmap...")
        try:
            data = {
                "header": {"stamp": str(costmap.header.stamp.sec)},
                "width": costmap.info.width,
                "height": costmap.info.height,
                "resolution": costmap.info.resolution,
                "data": list(costmap.data)
            }
            response = requests.post(f"{CLOUD_SERVER_URL}/upload_map", json=data)
            response.raise_for_status()
            self.get_logger().info("Costmap uploaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to upload costmap: {e}")
    
    def download_costmap(self):
        self.get_logger().info("Downloading latest costmap...")
        try:
            response = requests.get(f"{CLOUD_SERVER_URL}/download_map")
            response.raise_for_status()
            data = response.json()
            
            costmap = OccupancyGrid()
            costmap.header.stamp = self.get_clock().now().to_msg()
            costmap.info.width = data["width"]
            costmap.info.height = data["height"]
            costmap.info.resolution = data["resolution"]
            costmap.data = data["data"]
            
            self.publisher.publish(costmap)
            self.get_logger().info("Updated local costmap.")
        except Exception as e:
            self.get_logger().error(f"Failed to download costmap: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CostmapCloudSync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
