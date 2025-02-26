import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav2_msgs.action import NavigateToPose  # Nav2 action for navigation
import math
import websockets
import json
import asyncio
import os
import requests
from dotenv import dotenv_values

class UGVClient(Node):
    def __init__(self):
        super().__init__('ugv_client')

        config = dotenv_values(dotenv_path='./src/ugv_client/ugv_client/.env')

        # Read UGV ID from .env file
        self.ugv_id = config['UGV_ID']  # Default to 'UGV_1' if not set
        
        # Create an action client for the NavigateToPose action
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Control server address
        self.control_server_address = config['CONTROL_SERVER']

        # WebSocket URL (Replace with your cloud server's WebSocket address)
        self.websocket_url = "ws://"+self.control_server_address
        # API URL for registration (Replace with your server's address)
        self.registration_url = f"http://{self.control_server_address}/api/ugv/{self.ugv_id}/registerSession"
        
        # Connect and register UGV
        self.websocket_connection = None
        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(self.connect_and_register_ugv())

    async def connect_and_register_ugv(self):
        """ Connect to WebSocket and listen for the connection ID from the server """
        while True:
            try:
                # Connect to the WebSocket server
                self.websocket_connection = await websockets.connect(self.websocket_url)
                self.get_logger().info(f"Connected to WebSocket server {self.websocket_url}")

                # Wait for the connection ID from the server
                connection_id = await self.websocket_connection.recv()
                self.get_logger().info(f"Received connection ID: {connection_id}")

                # Call the HTTP registration API with the connection ID
                registration_payload = {'sessionId': connection_id}
                response = requests.put(self.registration_url, json=registration_payload)

                if response.status_code == 200:
                    self.get_logger().info(f"UGV {self.ugv_id} successfully registered.")
                else:
                    self.get_logger().error(f"Failed to register UGV {self.ugv_id}. HTTP Status: {response.status_code}")
                
                # Start listening for messages
                await self.listen_for_goals()
            except Exception as e:
                self.get_logger().error(f"Connection error: {e}. Retrying...")
                await asyncio.sleep(5)

    async def listen_for_goals(self):
        """ Listen for navigation goal messages from WebSocket server """
        while True:
            try:
                # Receive a message with navigation goal coordinates
                message = await self.websocket_connection.recv()
                self.get_logger().info(f"Received message: {message}")

                mesg = json.loads(message)

                if mesg["request_type"] == "navigate":
                    x = mesg['coordinates']['x']
                    y = mesg['coordinates']['y']
                    theta = mesg['coordinates']['theta']
                    # Send the goal to the Nav2 action server
                    await self.send_nav2_goal(x, y, theta)
                elif mesg["request_type"] == 'heartbeat':
                    response = {
                        "response_type": "heartbeat",
                        "status": "active",
                        "ugv_id": self.ugv_id
                    }
                    try:
                        await self.websocket_connection.send(json.dumps(response))
                    except Exception as e:
                        self.get_logger().error(f"Error sending heartbeat: {e}")
            except Exception as e:
                self.get_logger().error(f"Error receiving or processing message: {e}")

    async def send_nav2_goal(self, x, y, theta):
        """ Send a navigation goal to the Nav2 action server """
        # Wait for the action server to be available
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return

        # Create a PoseStamped message for the goal
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position = Point(x=x, y=y, z=0.0)
        goal_pose.pose.orientation = Quaternion(
            z=math.sin(theta / 2),
            w=math.cos(theta / 2)
        )

        # Create the goal message for the NavigateToPose action
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Send the goal to the action server
        self.get_logger().info(f"Sending navigation goal: x={x}, y={y}, theta={theta}")
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        

    def goal_response_callback(self, future):
        """ Callback for when the goal is accepted or rejected by the action server """
        self.get_logger().info("inside goal_response_callback")
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by the action server!")
            return

        self.get_logger().info("Goal accepted by the action server!")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """ Callback for when the navigation task is completed """
        self.get_logger().info("inside get_result_callback")
        result = future.result().result
        self.get_logger().info(f"Navigation completed with result: {result}")
        self.on_navigation_completed()

    def feedback_callback(self):
        """ Callback for receiving feedback during navigation """
        self.get_logger().info("inside feedback_callback")
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Current position: {feedback.current_pose}")

    def on_navigation_completed(self):
        """ Callback to be executed once navigation is completed """
        self.get_logger().info("inside on_navigation_completed")
        self.get_logger().info("Executing post-navigation tasks...")
        # Add your post-navigation logic here
        # For example, send a message back to the WebSocket server

def main(args=None):
    rclpy.init(args=args)
    client_node = UGVClient()
    rclpy.spin(client_node)

    # Shutdown the node after execution
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()