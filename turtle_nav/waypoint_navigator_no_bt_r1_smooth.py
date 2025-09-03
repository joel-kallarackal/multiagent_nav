import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

### Sinusoidal
import numpy as np
import matplotlib.pyplot as plt
import math


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

p1 = (-2.1270616951528085, -2.7723084523012593)
# p2 = (0.8108174780823566, -2.232505364315845)
p2 = (2.745143914068928, -2.0507380772115447)

WAYPOINTS = generate_sinusoidal_between_points(p1, p2, num_points=20, amplitude=0.5, frequency=2)
# WAYPOINTS = WAYPOINTS[:-6]
# WAYPOINTS = [(0.8108174780823566, -2.232505364315845)]

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator_r4')

        self.compute_path_client = ActionClient(self, ComputePathToPose, '/robot1/compute_path_to_pose')
        self.follow_path_client = ActionClient(self, FollowPath, '/robot1/follow_path')

        self.current_index = 0
        self.get_logger().info('Waiting for action servers...')
        self.compute_path_client.wait_for_server()
        self.follow_path_client.wait_for_server()
        self.get_logger().info('Connected to action servers.')
        
        self.timer1 = self.create_timer(0.1, self.waypoint_timer)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        self.pose = None
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/robot1/amcl_pose', self.pose_cb, qos_profile)

        self.percentage = 0.0
        self.waypoint_sent = False
        self.progress_updated_for_new_waypoint = False

        self.send_next_waypoint()

    def pose_cb(self, msg):
        self.pose = msg.pose.pose
        self.update_path_progress(self.pose)
    
    def euclidean_distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def update_path_progress(self, current_pose):
        if not hasattr(self, "current_path") or not self.current_path.poses:
            return
        
        # Precompute total path length once
        if not hasattr(self, "total_path_length"):
            self.total_path_length = 0.0
            for i in range(1, len(self.current_path.poses)):
                p1 = self.current_path.poses[i-1].pose.position
                p2 = self.current_path.poses[i].pose.position
                self.total_path_length += self.euclidean_distance(p1, p2)
        
        # Find closest point on path
        closest_idx = min(
            range(len(self.current_path.poses)),
            key=lambda i: self.euclidean_distance(
                current_pose.position, self.current_path.poses[i].pose.position
            )
        )
        
        # Distance traveled = sum of segments up to closest_idx
        traveled = 0.0
        for i in range(1, closest_idx + 1):
            p1 = self.current_path.poses[i-1].pose.position
            p2 = self.current_path.poses[i].pose.position
            traveled += self.euclidean_distance(p1, p2)
        
        self.percentage = traveled / self.total_path_length * 100.0
        if self.progress_updated_for_new_waypoint ==  False:
            self.progress_updated_for_new_waypoint = True
            self.waypoint_sent  = False
        self.get_logger().warn(f"Path progress: {self.percentage:.2f}%")


    def waypoint_timer(self):
        if self.percentage>20 and self.waypoint_sent == False:
            self.current_index+=1
            self.send_next_waypoint()
            self.waypoint_sent = True
            self.progress_updated_for_new_waypoint = False

    def send_next_waypoint(self):
        if self.current_index >= len(WAYPOINTS):
            self.get_logger().info('✅ All waypoints reached.')
            return

        x, y = WAYPOINTS[self.current_index]
        self.get_logger().info(f'📍 Sending waypoint {self.current_index + 1}: ({x}, {y})')

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = PoseStamped()
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.pose.position.x = x
        goal_msg.goal.pose.position.y = y
        goal_msg.goal.pose.orientation.w = 1.0  # Face forward

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
        self.current_path = result.path

        self.get_logger().info(f'🗺️ Got path with {len(path.poses)} poses. Sending to controller...')

        if not path.poses:
            self.get_logger().warn('⚠️ Empty path received. Skipping to next waypoint.')
            self.current_index += 1
            self.send_next_waypoint()
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
        result = future.result()
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('✅ Reached waypoint.')
            self.current_index += 1
            self.send_next_waypoint()
            self.waypoint_sent = True
            self.progress_updated_for_new_waypoint = False
            
        # self.send_next_waypoint()
        # self.waypoint_sent = True
        # self.progress_updated_for_new_waypoint = False


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
