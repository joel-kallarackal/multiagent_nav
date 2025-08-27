from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose, FollowPath
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from collections import deque
import rclpy

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid

"""
Game Status : IDLE, RUNNING, STOPPED
"""

class Pursuer(Node):
    def __init__(self):
        super().__init__('pursuer')

        self.evader_pose = None
        self.pursuer_pose = None
        self.global_costmap = None
        self.current_index = 0

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.evader_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/robot1/amcl_pose', self.evader_pose_cb, qos_profile)
        self.pursuer_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/robot2/amcl_pose', self.pursuer_pose_cb, qos_profile)
        self.game_status_sub = self.create_subscription(
            String, '/game_status', self.game_status_cb, 10)
        
        self.compute_path_client = ActionClient(self, ComputePathToPose, '/robot2/compute_path_to_pose')
        self.follow_path_client = ActionClient(self, FollowPath, '/robot2/follow_path')

        self.max_queue_size = 10 # Load from params file
        self.waypoint_stack = deque(maxlen=self.max_queue_size)        

        self.get_logger().info("Chauffeur on the road!")
        self.count = 0
        self.timeout = 10

        self.timer = self.create_timer(1.0, self.game_timer)
        self.pursuer_goal_status = 0 # 0 - No Goal, 1 - Goal Received
        self.status = None

    def game_timer(self):
        if self.status == "RUNNING" and self.pursuer_goal_status == 0:
            self.count+=1

        # Send the next waypoint if there is no goal sent for a while
        if self.count>=self.timeout:
            self.count=0
            self.send_next_waypoint()

    def evader_pose_cb(self, msg):
        if self.evader_pose==None:
            self.evader_pose = msg.pose.pose
            x, y = self.evader_pose.position.x, self.evader_pose.position.y
            self.waypoint_stack.append((x,y))
            self.send_next_waypoint()
        else:
            self.evader_pose = msg.pose.pose
            x, y = self.evader_pose.position.x, self.evader_pose.position.y
            self.waypoint_stack.append((x,y))

    def pursuer_pose_cb(self, msg):
        if self.pursuer_pose==None:
            self.get_logger().info("Human sees the chauffeur charging towards him")
        self.pursuer_pose = msg.pose.pose

    def game_status_cb(self, msg):
        self.status = msg.data

    def send_next_waypoint(self):
        if len(self.waypoint_stack) == 0:
            self.pursuer_goal_status = 0
            return

        self.pursuer_goal_status = 1
        self.count=0
        
        x, y = self.waypoint_stack.pop()
        self.get_logger().info(f'Sending waypoint {self.current_index + 1}: ({x}, {y})')

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
            self.get_logger().error('Path goal rejected.')
            self.pursuer_goal_status=0
            return

        self.get_logger().info('Path goal accepted. Waiting for result...')
        goal_handle.get_result_async().add_done_callback(self.send_to_controller)

    def send_to_controller(self, future):
        result = future.result().result
        path = result.path
        self.get_logger().info(f'Got path with {len(path.poses)} poses. Sending to controller...')

        if not path.poses:
            self.get_logger().warn('Empty path received. Skipping to next waypoint.')
            self.current_index += 1
            self.send_next_waypoint()
            self.pursuer_goal_status=0
            return

        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        future = self.follow_path_client.send_goal_async(follow_goal)
        future.add_done_callback(self.follow_path_response_callback)

    def follow_path_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal was rejected.')
            self.pursuer_goal_status=0
            return

        self.get_logger().info('FollowPath goal accepted. Following path...')
        goal_handle.get_result_async().add_done_callback(self.on_path_followed)

    def on_path_followed(self, future):
        self.get_logger().info('Reached waypoint.')
        self.current_index += 1
        self.send_next_waypoint()

def main():
    rclpy.init()
    node = Pursuer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()