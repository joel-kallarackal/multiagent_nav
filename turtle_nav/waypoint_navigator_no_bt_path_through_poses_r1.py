import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath, ComputePathThroughPoses
from nav_msgs.msg import Path
from rclpy.action import ActionClient

### Sinusoidal
import numpy as np
import matplotlib.pyplot as plt

def generate_sinusoidal_between_points(p1, p2, num_points=100, amplitude=1, frequency=1):
    """
    Generates sinusoidal waypoints along the line between two points.

    Parameters:
        p1 (tuple): (x1, y1) start point
        p2 (tuple): (x2, y2) end point
        num_points (int): Number of waypoints
        amplitude (float): Amplitude of the sine wave
        frequency (float): Number of sine cycles along the line

    Returns:
        waypoints (list of tuples): List of (x, y) waypoints
    """
    x1, y1 = p1
    x2, y2 = p2

    # Line vector and length
    line_vec = np.array([x2 - x1, y2 - y1])
    line_length = np.linalg.norm(line_vec)
    line_unit = line_vec / line_length

    # Perpendicular unit vector
    perp_unit = np.array([-line_unit[1], line_unit[0]])

    # Sample distances along the line
    t = np.linspace(0, 1, num_points)
    base_points = np.array([x1, y1]) + np.outer(t, line_vec)

    # Apply sinusoidal offset along perpendicular direction
    offsets = amplitude * np.sin(2 * np.pi * frequency * t)
    waypoints = base_points + np.outer(offsets, perp_unit)

    return waypoints.tolist()


# p1 = (-2.1270616951528085, -2.7723084523012593)
# p2 = (2.745143914068928, -2.0507380772115447)

p1 = (13.779971121307288, 17.244540004791123)
p2 = (13.671288153615949, 20.31729862980733)

WAYPOINTS = generate_sinusoidal_between_points(p1, p2, num_points=25, amplitude=0.5, frequency=2)

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator_r1')

        self.compute_path_client = ActionClient(self, ComputePathThroughPoses, '/robot1/compute_path_through_poses')
        self.follow_path_client = ActionClient(self, FollowPath, '/robot1/follow_path')

        self.current_index = 0
        self.get_logger().info('Waiting for action servers...')
        self.compute_path_client.wait_for_server()
        self.follow_path_client.wait_for_server()
        self.get_logger().info('Connected to action servers.')

        self.compute_path()

    def compute_path(self):
        if len(WAYPOINTS) == 0:
            self.get_logger().info('Waypoint set passed is empty.')
            return

        self.get_logger().info('Computing path...')
        goal_msg = ComputePathThroughPoses.Goal()
        goal_msg.goals = []

        for (x, y) in WAYPOINTS:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            goal_msg.goals.append(pose)

        future = self.compute_path_client.send_goal_async(goal_msg)
        future.add_done_callback(self.path_response_callback)

    def path_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Path goal rejected.')
            return

        self.get_logger().info('🛣️ Path goal accepted. Waiting for result...')
        goal_handle.get_result_async().add_done_callback(self.send_to_controller)

    def send_to_controller(self, future):
        result = future.result().result
        path = result.path
        self.get_logger().info(f'🗺️ Got full path with {len(path.poses)} poses. Sending to controller...')

        if not path.poses:
            self.get_logger().warn('⚠️ Empty path received.')
            return

        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        future = self.follow_path_client.send_goal_async(follow_goal)
        future.add_done_callback(self.follow_path_response_callback)


    def follow_path_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ FollowPath goal was rejected.')
            return

        self.get_logger().info('🚶 FollowPath goal accepted. Following path...')
        goal_handle.get_result_async().add_done_callback(self.on_path_followed)

    def on_path_followed(self, future):
        self.get_logger().info('✅ Reached goal.')
        self.current_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
