import rclpy
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from typing import List, Tuple


from actions.action import PlannerAction
from .point_cloud import PointCloud
import numpy as np
from .lib.map_generation import random_map
from interfaces.msg import State as StateMessage
from random import seed

TIMER_PERIOD = 2  # seconds

MAP_POINT_PER_METER = 25

ROBOT_ID = 1
ROBOT_WIDTH = 0.5
NUMBER_OF_ANGLES = 4
REMAINING_THRESHOLD = 1.0

RANDOM_SEED = 100
np.random.seed(RANDOM_SEED)
seed(RANDOM_SEED)


class MinimalPublisher(Node):

    previous_steps: List[Tuple[float, float, float]] = [(0, 0, 0)]
    path: List[Tuple[float, float, float]]
    pc = PointCloud

    @property
    def position(self):
        return self.previous_steps[0]

    def __init__(self):
        super().__init__('robot_node')
        self.pc = PointCloud()
        self.previous_steps = [(0, 0, 0)]
        self.path = []

        self.MAP_SIZE = (5, 5, 0.001)
        self.NUMBER_POINTS = 5000

        traversable_points = random_map(delta_x=self.MAP_SIZE[0], delta_y=self.MAP_SIZE[1], delta_z=self.MAP_SIZE[2], n=self.NUMBER_POINTS)

        self.pc.extend(traversable_points)

        # create publishers
        # self.pub_position = self.create_publisher(sensor_msgs, 'position', 10)
        self.pub_point_cloud = self.create_publisher(StateMessage, 'point_cloud', 10)

        # create actions
        self._action_client = ActionClient(self, PlannerAction, "mission")
        
        # create timers
        self.create_timer(TIMER_PERIOD, self.timer_callback)
        self.create_timer(TIMER_PERIOD, self.drive_robot)


        # start planner
        self.timer_action = self.create_timer(5, self.call_action)


    def call_action(self):
        future = self.send_goal()
    

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {feedback.feedback}")
    

    def goal_response_callback(self, future):
        # Get handle for the goal we just sent
        goal_handle = future.result()

        # Return early if goal is rejected
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Use goal handle to request the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result

        self.path = [(p.x, p.y, p.z) for p in result.result]

        # Log result and shut down ROS 2 cleanly
        self.get_logger().info(f"Result: {np.asarray(self.path)[:min(10, len(self.path))]}")

        # rclpy.shutdown()
    
    def drive_robot(self):

        for _ in range(10):
            while self.path:
                pos = self.path.pop(0)

                if pos not in self.previous_steps:
                    self.previous_steps.append(pos)
                    break
                


    def send_goal(self):
        goal_msg = PlannerAction.Goal()

        goal_msg.robot_id = ROBOT_ID
        goal_msg.robot_width = ROBOT_WIDTH
        goal_msg.number_of_angles = NUMBER_OF_ANGLES
        goal_msg.remaining_threshold = REMAINING_THRESHOLD


        self._action_client.wait_for_server()

        # Returns future to goal handle; client runs feedback_callback after sending the goal
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        # Register a callback for when future is complete (i.e. server accepts or rejects goal request)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        
    def timer_callback(self):

        point_cloud_message = self.pc.serialize()
        payload = StateMessage()
        payload.traversable = point_cloud_message
        payload.inaccessible = PointCloud().serialize()
        payload.robot_id = 10
        payload.previous_positions = []

        print(f"previous steps: {np.asarray(self.previous_steps)}")

        pnts = []
        for p_v in np.asarray(self.previous_steps):
            p = Point()
            p.x, p.y, p.z = p_v.astype(float)
            pnts.append(p)
        
        payload.previous_positions = pnts

        self.pub_point_cloud.publish(payload)

        self.get_logger().info(f"Point Cloud published")


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()