import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import asyncio
from concurrent.futures import ThreadPoolExecutor
import websockets
import json
import requests
from dotenv import dotenv_values
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav2_msgs.action import NavigateToPose
import math

class UGVClient(Node):
    def __init__(self):
        super().__init__('ugv_client')
        config = dotenv_values(dotenv_path='./src/ugv_client/ugv_client/.env')
        self.ugv_id = config['UGV_ID']
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.control_server_address = config['CONTROL_SERVER']
        self.websocket_url = f"ws://{self.control_server_address}"
        self.registration_url = f"http://{self.control_server_address}/api/ugv/{self.ugv_id}/registerSession"
        self.websocket_connection = None

    async def connect_and_register_ugv(self):
        while True:
            try:
                self.websocket_connection = await websockets.connect(self.websocket_url)
                self.get_logger().info(f"Connected to WebSocket server")
                connection_id = await self.websocket_connection.recv()
                registration_payload = {'sessionId': connection_id}
                response = requests.put(self.registration_url, json=registration_payload)
                if response.status_code == 200:
                    self.get_logger().info(f"UGV {self.ugv_id} registered.")
                else:
                    self.get_logger().error(f"Registration failed: {response.status_code}")
                await self.listen_for_goals()
            except Exception as e:
                self.get_logger().error(f"Connection error: {e}. Retrying...")
                await asyncio.sleep(5)

    async def listen_for_goals(self):
        while True:
            try:
                message = await self.websocket_connection.recv()
                mesg = json.loads(message)
                if mesg["request_type"] == "navigate":
                    self.current_navigation_id = mesg["navigation_id"]
                    x = mesg['coordinates']['x']
                    y = mesg['coordinates']['y']
                    theta = mesg['coordinates']['theta']
                    await self.send_nav2_goal(x, y, theta)
                elif mesg["request_type"] == 'heartbeat':
                    response = {
                        "response_type": "heartbeat",
                        "status": "active",
                        "ugv_id": self.ugv_id
                    }
                    await self.websocket_connection.send(json.dumps(response))
            except Exception as e:
                self.get_logger().error(f"Error processing message: {e}")

    async def send_nav2_goal(self, x, y, theta):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position = Point(x=x, y=y, z=0.0)
        goal_pose.pose.orientation = Quaternion(
            z=math.sin(theta / 2),
            w=math.cos(theta / 2)
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(f"Sending navigation goal: x={x}, y={y}, theta={theta}")
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by the action server!")
            return

        self.get_logger().info("Goal accepted by the action server!")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    async def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Navigation completed with result: {result}")
        await self.on_navigation_completed()

    async def on_navigation_completed(self):
        self.get_logger().info("Executing post-navigation tasks...")

        payload = {
            "response_type":"navigation_callback",
            "request_id":self.current_navigation_id
        }

        self.current_navigation_id = ""

        self.get_logger().info("navigation completed, sending callback to server....")

        await self.websocket_connection.send(json.dumps(payload))


def main(args=None):
    rclpy.init(args=args)
    node = UGVClient()

    # Use a MultiThreadedExecutor to handle ROS2 callbacks and asyncio tasks
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Run the asyncio loop in a separate thread
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_in_executor(None, lambda: loop.run_until_complete(node.connect_and_register_ugv()))

    try:
        # Spin the node to handle ROS2 callbacks
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        loop.close()

if __name__ == '__main__':
    main()