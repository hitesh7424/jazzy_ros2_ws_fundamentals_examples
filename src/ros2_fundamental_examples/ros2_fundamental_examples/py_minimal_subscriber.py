#! /usr/bin/env python3

"""
Description:
This ROS2 node subscribes to a topic publishing 'hello world' messages.

Publishing Topics:
    None

Subscription Topics:
    /py_example_topic - std_msgs/String

Usage:
    This node listens to the topic '/py_example_topic' and logs the received messages.

Author: Hitesh
Date: December 21, 2025
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPySubscriber(Node):
    """
    A minimal ROS2 subscriber node that listens to the '/py_example_topic' topic.

    Attributes:
        subscriber_1: The subscription object for the '/py_example_topic' topic.
    """

    def __init__(self):
        """
        Initializes the MinimalPySubscriber node and sets up the subscription.
        """
        super().__init__("minimal_py_subscriber")

        # Create a subscription to the '/py_example_topic' topic
        self.subscriber_1 = self.create_subscription(
            String, "py_example_topic", self.listener_callback, 10
        )

    def listener_callback(self, msg):
        """
        Callback function that gets executed when a message is received on the '/py_example_topic' topic.

        Args:
            msg (std_msgs.msg.String): The message received from the topic.
        """
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    """
    The main entry point for the ROS2 node.

    This function initializes the ROS2 Python client library, creates the subscriber node,
    and keeps it spinning to listen for messages.

    Args:
        args: Command-line arguments passed to the node (default: None).
    """
    rclpy.init(args=args)

    # Create an instance of the MinimalPySubscriber node
    minimal_py_subscriber = MinimalPySubscriber()

    # Keep the node running to listen for messages
    rclpy.spin(minimal_py_subscriber)

    # Destroy the node explicitly (optional cleanup)
    minimal_py_subscriber.destroy_node()

    # Shutdown the ROS2 Python client library
    rclpy.shutdown()


if __name__ == "__main__":
    main()
