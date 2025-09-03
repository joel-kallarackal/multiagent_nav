from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose, FollowPath
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
import random
from collections import deque
from nav_msgs.msg import OccupancyGrid, Path
import math
import numpy as np
import rclpy
import random
import time

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

"""
Game Status : IDLE, RUNNING, STOPPED
"""

class Evader(Node):
    def __init__(self):
        super().__init__('evader')
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
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid, '/robot1/global_costmap/costmap', self.costmap_cb, qos_profile)
        self.game_status_sub = self.create_subscription(
            String, '/game_status', self.game_status_cb, 10)
        self.plan_sub = self.create_subscription(
            Path, '/robot1/plan', self.plan_cb, 10)
        
        self.compute_path_client = ActionClient(self, ComputePathToPose, '/robot1/compute_path_to_pose')
        self.follow_path_client = ActionClient(self, FollowPath, '/robot1/follow_path')

        self.max_queue_size = 10 # Load from params file
        self.waypoint_stack = deque(maxlen=self.max_queue_size)

        self.get_logger().info("Human on the road!")
        self.count = 0
        self.timeout = 10

        self.timer1 = self.create_timer(0.1, self.waypoint_timer)
        self.timer2 = self.create_timer(1, self.send_goal_timer) # timer2 should run at a lower frequency than timer1

        self.evader_goal_status = 0 # 0 - No Goal, 1 - Goal Received
        self.status = None

        self.current_path = None
        self.remaining_path = None

    def waypoint_timer(self):
        if self.status == "RUNNING" and self.evader_goal_status == 0:
            self.count+=1
        
        if self.pursuer_pose!=None and self.evader_pose!=None:
            waypoint = self.calculate_waypoint()
            if waypoint is not None:
                self.waypoint_stack.append(waypoint)

        # # Send the next waypoint if there is no goal sent for a while
        # if self.count>=self.timeout:
        #     self.count=0
        #     if len(self.waypoint_stack)==0:
        #         self.waypoint_stack.append()
        #     self.send_next_waypoint()
    
    def send_goal_timer(self):
        if self.current_path is not None and self.remaining_path is not None:
            self.get_logger().info(f"{len(self.remaining_path.poses)}")

        if self.status == "RUNNING":
            self.send_next_waypoint()

    def evader_pose_cb(self, msg):
        self.evader_pose = msg.pose.pose

    def pursuer_pose_cb(self, msg):
        if self.pursuer_pose==None:
            self.get_logger().info("Human sees the chauffeur charging towards him!")
            # self.pursuer_pose = msg.pose.pose
            # waypoint = self.calculate_waypoint()
            # if waypoint is not None:
            #     self.waypoint_stack.append(waypoint)
            #     self.send_next_waypoint()
        
        self.pursuer_pose = msg.pose.pose
        
    def costmap_cb(self, msg: OccupancyGrid):
        self.global_costmap = msg

    def game_status_cb(self, msg):
        self.status = msg.data
    
    def plan_cb(self, msg):
        self.remaining_path = msg
    
    def calculate_waypoint(self, num_samples=250, cost_threshold=50):
        if self.global_costmap is None or self.evader_pose is None or self.pursuer_pose is None:
            self.get_logger().warn("Waypoint None")
            return
        free_cells = []
        width = self.global_costmap.info.width
        height = self.global_costmap.info.height
        resolution = self.global_costmap.info.resolution
        origin_x = self.global_costmap.info.origin.position.x
        origin_y = self.global_costmap.info.origin.position.y
        data = self.global_costmap.data

        chaf_pos_vec = np.array([self.pursuer_pose.position.x, self.pursuer_pose.position.y])
        ev_pos_vec = np.array([self.evader_pose.position.x, self.evader_pose.position.y])

        a_hat = (ev_pos_vec-chaf_pos_vec)/np.linalg.norm(ev_pos_vec-chaf_pos_vec)
        b1 = np.sqrt(1/(1+a_hat[0]**2/a_hat[1]**2))
        b2 = -b1*a_hat[0]/a_hat[1]
        b_hat = np.array([b1, b2])
        
        min_r = 0.4
        max_r = 1.2

        # Uniformly sample unoccupied points from the costmap
        for _ in range(num_samples):
            idx = random.randint(0, width * height - 1)
            if data[idx] >= 0 and data[idx] <= cost_threshold:
                x = origin_x + (idx % width) * resolution
                y = origin_y + (idx // width) * resolution

                # Only if it is in front of the evader bot
                if np.dot(a_hat,np.array([x,y])-ev_pos_vec)>0 and np.linalg.norm(np.array([x,y])-ev_pos_vec)>min_r and np.linalg.norm(np.array([x,y])-ev_pos_vec)<max_r:
                    free_cells.append((x, y))
        
        # Go through every cell
        # for 4
        # Sample a example goal point
        # Probability distribution : p(x) = (b - cos(x)) / (b*pi - 2) 
        # p_inv(x) = arccos(b - (b*pi-2)*x)
        b = 1.05
        x_list = np.array([random.uniform((b-1)/(b*np.pi-2), b/(b*np.pi-2)) for i in range(50)])

        theta1 = np.arccos(b-(b*np.pi-2)*x_list)
        theta2 = -np.arccos(b-(b*np.pi-2)*x_list)

        theta = np.concatenate((theta1, theta2))

        theta = np.array([random.uniform(-np.pi/3, np.pi/3) for i in range(100)])
        
        r = 1
        example_waypoints = ev_pos_vec + r*np.cos(np.pi/2 - theta)[:, None]*b_hat + r*np.sin(np.pi/2 - theta)[:, None]*a_hat

        scores = []
        for waypoint in free_cells:
            for example_waypoint in example_waypoints:
                score = 1*np.linalg.norm(waypoint-ev_pos_vec) + 1/(np.dot(example_waypoint/np.linalg.norm(example_waypoint), (waypoint-ev_pos_vec)/np.linalg.norm(waypoint-ev_pos_vec)))
                scores.append((waypoint,score))
        
        scores.sort(key=lambda s: s[1], reverse=True)
        if len(scores)>0:
            best_waypoint = scores[0][0]
            return best_waypoint
        else:
            return None

        

    def send_next_waypoint(self):
        if len(self.waypoint_stack) == 0:
            self.get_logger().info("No waypoints available!")
            self.evader_goal_status = 0
            return

        self.evader_goal_status = 1
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
            self.evader_goal_status=0
            return

        self.get_logger().info('Path goal accepted. Waiting for result...')
        goal_handle.get_result_async().add_done_callback(self.send_to_controller)

    def send_to_controller(self, future):
        result = future.result().result
        path = result.path
        self.current_path = path
        self.get_logger().info(f'Got path with {len(path.poses)} poses. Sending to controller...')

        if not path.poses:
            self.get_logger().warn('Empty path received. Skipping to next waypoint.')
            self.current_index += 1
            self.send_next_waypoint()
            self.evader_goal_status=0
            return

        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        future = self.follow_path_client.send_goal_async(follow_goal)
        future.add_done_callback(self.follow_path_response_callback)

    def follow_path_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal was rejected.')
            self.evader_goal_status=0
            return

        self.get_logger().info('FollowPath goal accepted. Following path...')
        goal_handle.get_result_async().add_done_callback(self.on_path_followed)

    def on_path_followed(self, future):
        self.get_logger().info('Reached waypoint.')
        self.current_index += 1
        if self.status == "RUNNING":
            self.send_next_waypoint()

    
def main():
    rclpy.init()
    node = Evader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()