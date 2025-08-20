import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient

WAYPOINTS = [
            (-1.1888759073286665, 0.9567395337275825),
            (-1.7573231267877698, 1.1763000837780282),
            (-2.0487147682992837, 0.6371434216059553),
            (-2.761993990854728, 0.7684627066245723),
            (-2.523020336341442, 1.5389058964368052),
            (-3.0333638445221593, 1.6243871917065487),
            (-3.575691666712014, 0.8540813600863703)
        ]

# WAYPOINTS = WAYPOINTS[::-1]

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator_r1')

        self.compute_path_client = ActionClient(self, ComputePathToPose, '/robot1/compute_path_to_pose')
        self.follow_path_client = ActionClient(self, FollowPath, '/robot1/follow_path')

        self.current_index = 0
        self.get_logger().info('Waiting for action servers...')
        self.compute_path_client.wait_for_server()
        self.follow_path_client.wait_for_server()
        self.get_logger().info('Connected to action servers.')

        self.replanning_enabled = False
        self.replan_interval = 2.0  # seconds
        self.current_goal_pose = None
        self.replan_timer = self.create_timer(self.replan_interval, self.replan_timer_callback)

        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.current_index >= len(WAYPOINTS):
            self.get_logger().info('✅ All waypoints reached.')
            return

        x, y = WAYPOINTS[self.current_index]
        self.get_logger().info(f'📍 Sending waypoint {self.current_index + 1}: ({x}, {y})')
        
        self.current_goal_pose = PoseStamped()
        self.current_goal_pose.header.frame_id = 'map'
        self.current_goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.current_goal_pose.pose.position.x = x
        self.current_goal_pose.pose.position.y = y
        self.current_goal_pose.pose.orientation.w = 1.0 # Face forward
        
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self.current_goal_pose

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
        self.replanning_enabled = True  # Start replanning once controller starts

        result = future.result().result
        path = result.path
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
        self.replanning_enabled = False  # Stop replanning once done
        self.current_goal_pose = None
        self.get_logger().info('✅ Reached waypoint.')
        self.current_index += 1
        self.send_next_waypoint()

    def replan_timer_callback(self):
        if not self.replanning_enabled or self.current_goal_pose is None:
            return

        self.get_logger().info('🔁 Replanning to current goal...')
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = self.current_goal_pose
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()

        future = self.compute_path_client.send_goal_async(goal_msg)
        future.add_done_callback(self.replan_path_callback)

    def replan_path_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('⚠️ Replan goal rejected.')
            return

        goal_handle.get_result_async().add_done_callback(self.send_replanned_path)

    def send_replanned_path(self, future):
        result = future.result().result
        path = result.path

        if not path.poses:
            self.get_logger().warn('⚠️ Empty replanned path. Skipping.')
            return

        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        self.follow_path_client.send_goal_async(follow_goal)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
