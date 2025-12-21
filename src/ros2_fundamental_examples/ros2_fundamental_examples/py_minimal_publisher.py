#! /usr/bin/env python3

"""
Description:
This ROS 2 Node periodically publishes "Hello World" messages to a topic.

Publishing Topics:
  - /py_example_topic (std_msgs/String): This channel contains the "Hello World" messages.

Subscription Topics:
  - None

Author: Hitesh
Date: December 21, 2025
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPyPublisher(Node):
    """
    A minimal ROS 2 publisher node that periodically publishes messages to a topic.
    """

    def __init__(self):
        """
        Initialize the publisher node and set up the timer for periodic publishing.
        """
        # Initialize the node with a name
        super().__init__("minimal_py_publisher")

        # Create a publisher on the topic with a queue size of 10 messages
        self.publisher_1 = self.create_publisher(String, "/py_example_topic", 10)

        # Create a timer with a period of 0.5 seconds to trigger the publishing of messages
        timer_period = 0.5  # Timer period in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize a counter variable for message content
        self.i = 0

    def timer_callback(self):
        """
        Callback function executed periodically by the timer to publish messages.
        """
        # Create a new string message object
        msg = String()

        # Set the message data with a counter
        msg.data = f"Hello World {self.i}"

        # Publish the message to the topic
        self.publisher_1.publish(msg)

        # Log a message indicating the message has been published
        self.get_logger().info(f"Publishing: {msg.data}")

        # Increment the counter for the next message
        self.i += 1


def main(args=None):
    """
    Main function to start the ROS 2 node.

    Args:
        args (List, optional): Command line arguments. Defaults to None.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the minimal publisher node
    minimal_py_publisher = MinimalPyPublisher()

    # Keep the node running to listen and respond
    rclpy.spin(minimal_py_publisher)

    # Destroy the node explicitly to release resources
    minimal_py_publisher.destroy_node()

    # Shutdown ROS 2 communication
    rclpy.shutdown()


if __name__ == "__main__":
    # Execute the main function if the script is run directly
    main()
