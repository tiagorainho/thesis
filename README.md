# Sasori



# How to integrate with ROS2 node

Since a ROS2 node is needed, we could simply use the already existing package ``Sasori`` to add that node. However, for better organization a new *package* should be created specific for the use case at hands:

```bash
ros2 pkg create --build-type ament_python --node-name new_node package_name
```

To *build* only the created package:

```bash
colcon build --packages-select package_name
```

Then, source the underlying setup file:

```bash
source install/local_setup.bash
```

And finally run it

```bash
ros2 run package_name new_node
```

## How to use

In this example, we can check all the code required to program a lawn-mower using this framework.

```python
import rclpy
from rclpy.node import Node
from interfaces.msg import State as StateMessage
from utils import blades_handler
from point_cloud import PointCloud

MAXIMUM_BUFFER = 2
ROBOT_WIDTH = 0.5

class BladeControllerNode(Node):

    def __init__(self):
        super().__init__('blade_controller')

        self.already_covered_points = PointCloud()

        # instantiate the subscribers and publishers
        self.state_subscriber = self.create_subscription(
            StateMessage,
            'state',
            self.state_listener_callback,
            MAXIMUM_BUFFER)

    def state_listener_callback(self, state):

        # get current goal area
        goal_pc = PointCloud.desserialize(state.data.goal_pc)

        # is outside the goal area
        if len(goal_pc.closest_neighbor(point=state.data.robot_position, max_radius=ROBOT_WIDTH)) == 0:
            blades_handler.shutdown()
            return

        # check if it has already trimmed that position
        if len(self.already_covered_points.closest_neighbor(point=state.data.robot_position, max_radius=ROBOT_WIDTH)) > 0:
            blades_handler.shutdown()
            return
        
        # else
        blades_handler.run()
        self.already_covered_points.extend(robot_position_with_fixed_reference)


def main(args=None):
    rclpy.init(args=args)

    example_node = BladeControllerNode()

    rclpy.spin(example_node)

    example_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

It should be noted that the published state and robot position has already been merged with all the other available robots. The developer does not need to bother with the state management.

# Display

To use the display:

Disable access control to the monitor
```bash
xhost +
```

Example:
```bash
rqt_graph
```
