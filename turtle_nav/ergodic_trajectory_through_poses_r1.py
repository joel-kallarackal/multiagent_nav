import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowPath, ComputePathThroughPoses
from nav_msgs.msg import Path
from rclpy.action import ActionClient
import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat
import os, csv

data = loadmat("/home/joel/biorobotics_lab/turtlebots_ws/src/turtle_nav/data/traj.mat")
trajectory = data["currTraj"]

scale_x = 0.1
scale_y = 0.1

center = trajectory.mean(axis=0)
# shift_from_center = [16.167286794795572, 27.445176474273982]
shift_from_center = [14.18414079168812, 28.446431472708326]
# shift_from_center = [14.31562817479311, 21.62429143244914]

scale_shift_trajectory = (trajectory - center) * np.array([scale_x, scale_y]) + shift_from_center

# scale_shift_trajectory = scale_shift_trajectory[:2500]
scale_shift_trajectory = np.concatenate([scale_shift_trajectory, [[13.909006882761878, 21.202070486069047]]])  # Final goal point set far away so that robot doesnt reach goal in between
sampled_trajectory = scale_shift_trajectory

WAYPOINTS = sampled_trajectory

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator_r1')

        self.compute_path_client = ActionClient(self, ComputePathThroughPoses, '/robot1/compute_path_through_poses')
        self.follow_path_client = ActionClient(self, FollowPath, '/robot1/follow_path')

        # Subscriber to AMCL
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot1/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        # Logging variables
        self.logged_poses = []
        self.log_file = os.path.expanduser(
            "~/biorobotics_lab/turtlebots_ws/src/turtle_nav/data/logged_poses.csv"
        )

        self.current_index = 0
        self.get_logger().info('Waiting for action servers...')
        self.compute_path_client.wait_for_server()
        self.follow_path_client.wait_for_server()
        self.get_logger().info('Connected to action servers.')

        self.compute_path()

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Callback to log AMCL poses in real-time."""
        pose = msg.pose.pose
        self.logged_poses.append([pose.position.x, pose.position.y])

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

        ##### Send to controller directly
        # path = Path()
        # path.header.frame_id = "map"   # Or "odom" depending on your setup
        # path.header.stamp = self.get_clock().now().to_msg()

        # # Fill in the path with PoseStamped waypoints
        # for x, y in WAYPOINTS:
        #     pose = PoseStamped()
        #     pose.header.frame_id = "map"
        #     pose.header.stamp = self.get_clock().now().to_msg()
        #     pose.pose.position.x = x
        #     pose.pose.position.y = y
        #     pose.pose.position.z = 0.0
        #     pose.pose.orientation.w = 1.0  # Facing forward
        #     path.poses.append(pose)
        #####

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
        result = future.result().result  # <- ACTION RESULT FUTURE
        self.get_logger().info(f"Result: {result.status}")
        self.get_logger().info('✅ Reached goal.')

        self.save_logged_poses()
        self.get_logger().info(f"Logged {len(self.logged_poses)} poses to {self.log_file}")

        self.current_index += 1

    def save_logged_poses(self):
        """Save all collected AMCL poses to a CSV file."""
        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
        with open(self.log_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x", "y"])
            writer.writerows(self.logged_poses)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
