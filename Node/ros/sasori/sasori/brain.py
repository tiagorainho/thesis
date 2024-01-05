import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from actions.action import PlannerAction
from rclpy.action.server import ServerGoalHandle
from geometry_msgs.msg import Point

import numpy as np
from .cpp_inward_spiral import InwardSpiralSolver
from .planner import Planner 
from .state import State
from .state_manager import StateManager
from .communication_node import CommunicationNode
from .orchestrator import Orchestrator
from .robot import Robot
from .point_cloud import PointCloud
from interfaces.msg import State as StateMessage
from .communication_node import StatePayload

# https://foxglove.dev/blog/creating-ros2-actions

ROBOT_ID = 10


class PlannerServer(Node):

    
    def point_cloud_listener_callback(self, msg):

        # self.robot.orchestrator.state.traversable_pc = PointCloud.deserialize(message = msg)
        

        if msg.robot_id == ROBOT_ID: #  self.robot.robot_id:
            self.state_manager.state.traversable_pc = PointCloud.deserialize(msg.traversable)
            self.state_manager.state.previous_positions = [(p.x, p.y, p.z) for p in msg.previous_positions]

            # print(self.state_manager.state.previous_positions)

            
        else:
            self.state_manager.communication_node.recv(StatePayload(
                identifier=msg.robot_id,
                point_cloud=PointCloud.deserialize(msg.traversable)
            ))
            
        self.get_logger().info(f"Robot {msg.robot_id}: State updated")
        
        # self.get_logger().info(f"Point Cloud of {msg.robot_id} received, path: {self.state_manager.state.previous_positions}")
    

    def __init__(self):
        super().__init__('planner_server')

        self.robot = None
        self.state_manager = StateManager(
            state = State(),
            communication_node=CommunicationNode(ROBOT_ID)
        )

        # subscriptions
        self.sub_point_cloud = self.create_subscription(
            StateMessage,
            'point_cloud',
            self.point_cloud_listener_callback,
            10)
        
        self._action_server = ActionServer(
            self,
            PlannerAction,
            'mission',
            self.execute_callback)

    def execute_callback(self, goal_handle: ServerGoalHandle):
        robot_id = goal_handle.request.robot_id
        robot_width = goal_handle.request.robot_width
        number_of_angles = goal_handle.request.number_of_angles
        remaining_threshold = goal_handle.request.remaining_threshold

        self.get_logger().info(f"Planning started: {goal_handle.request}")

        # for now
        self.starting_position = [0, 0, 0]

        # instantiate main components
        robot_solver = InwardSpiralSolver(
            robot_width=robot_width,
            n_angles=number_of_angles,
            remaining_threshold=remaining_threshold)
        robot_planner = Planner(cpp_solver=robot_solver)
        
        robot_orchestrator = Orchestrator(
            planner=robot_planner,
            state_manager=self.state_manager
        )

        # create the robot
        self.robot = Robot(position=np.asarray(self.starting_position), orchestrator=robot_orchestrator)

        self.robot.orchestrator.state.inaccessible_pc.extend(np.asarray([]))

        self.robot.run()

        # print(self.robot.orchestrator.state.traversable_pc.serialize())

        goal_handle.succeed()
        result = PlannerAction.Result()

        plan_path = []
        for coord in self.robot.orchestrator.planner.plan.path:
            p = Point()
            p.x, p.y, p.z = float(coord[0]), float(coord[1]), float(coord[2])
            plan_path.append(p)

        result.result = plan_path

        self.get_logger().info(f"Robot {self.robot.robot_id} has {len(plan_path)} steps")

        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = PlannerServer()
    
    rclpy.spin(action_server)


if __name__ == '__main__':
    main()


# ros2 action send_goal mission actions/action/PlannerAction "{robot_id: 1}"