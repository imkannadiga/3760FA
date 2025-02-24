import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Quaternion
import math
import websockets
import json
import asyncio
import os
import requests
from dotenv import load_dotenv

class UGVClient(Node):
    def __init__(self):
        super().__init__('ugv_client')
        
        load_dotenv()

        # Read UGV ID from .env file
        self.ugv_id = os.getenv('UGV_ID')  # Default to 'UGV_1' if not set
        
        # Create a publisher for the /goal_pose topic
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Control server address
        self.control_server_address = os.getenv('CONTROL_SERVER')

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

                # Parse the coordinates from the WebSocket message
                mesg = json.loads(message)
                if(mesg['request_type'] == 'navigate') {
                    x = mesg['coordinates']['X']
                    y = mesg['coordinates']['Y']
                    theta = mesg['coordinates']['theta']
                    # Create and publish the goal pose
                    self.publish_goal(x, y, theta)
                } else if (mesg['request_type'] == 'heartbeat') {
                    response = {
                        "response_type" : "heartbeat",
                        "status" : "active"
                        "ugv_id" : self.ugv_id
                    }
                    try:
                        this.websocket_connection.send(response);
                    except e:
                        self.get_logger().error(f"Error sending heartbeat {e}")
                }
            except Exception as e:
                self.get_logger().error(f"Error receiving or processing message: {e}")

    def publish_goal(self, x, y, theta):
        """ Create a PoseStamped message and publish to /goal_pose topic """
        goal_pose = PoseStamped()

        # Set the position
        goal_pose.pose.position = Point(x=x, y=y, z=0.0)

        # Set the orientation (convert theta to quaternion)
        quat = Quaternion()
        quat.z = math.sin(theta / 2)
        quat.w = math.cos(theta / 2)
        goal_pose.pose.orientation = quat

        goal_pose.header.frame_id='map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        # Publish the goal pose
        self.goal_pose_publisher.publish(goal_pose)
        self.get_logger().info(f"Published goal: x={x}, y={y}, theta={theta}")

def main(args=None):
    rclpy.init(args=args)
    client_node = UGVClient()
    rclpy.spin(client_node)

    # Shutdown the node after execution
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
