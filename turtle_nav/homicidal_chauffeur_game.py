from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose, FollowPath
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
from std_msgs.msg import String
import rclpy

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class HomicidalChauffeur(Node):
    def __init__(self):
        super().__init__('homicidal_chauffeur')
        
        self.human_pose = None
        self.chauffeur_pose = None 

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.human_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/robot1/amcl_pose', self.human_pose_cb, qos_profile)
        self.chauffeur_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/robot2/amcl_pose', self.chauffeur_pose_cb, qos_profile)
        
        self.game_status_pub = self.create_publisher(String, '/game_status', 10)
        
        self.human_compute_path_client = ActionClient(self, ComputePathToPose, '/robot1/compute_path_to_pose')
        self.human_follow_path_client = ActionClient(self, FollowPath, '/robot1/follow_path')

        self.chauffeur_compute_path_client = ActionClient(self, ComputePathToPose, '/robot2/compute_path_to_pose')
        self.chauffeur_follow_path_client = ActionClient(self, FollowPath, '/robot2/follow_path')

        # Periodic game updates
        self.timer = self.create_timer(0.1, self.game_timer)

        self.dist_thresh = 0.4 # Pull from params file

        self.game_initialized = False
        self.game_over = False
        self.get_logger().info("Homicidal Chauffeur Game Manager started!")

    def human_pose_cb(self, msg):
        self.human_pose = msg.pose.pose

    def chauffeur_pose_cb(self, msg):
        self.chauffeur_pose = msg.pose.pose

    def game_timer(self):
        msg = String()
        if self.human_pose is None or self.chauffeur_pose is None:
            msg.data = "IDLE"
            self.game_status_pub.publish(msg)
            return
        
        if self.game_over:
            msg.data = "STOPPED"
        else:
            msg.data = "RUNNING"

        self.game_status_pub.publish(msg)
        
        chaf_pos_vec = np.array([self.chauffeur_pose.position.x, self.chauffeur_pose.position.y])
        ev_pos_vec = np.array([self.human_pose.position.x, self.human_pose.position.y])

        dist = np.linalg.norm(chaf_pos_vec-ev_pos_vec)

        if dist<self.dist_thresh:
            if not self.game_over:
                self.get_logger().warn("Chauffeur : HA! HA! HA! HA! HA!")
            self.game_over = True

def main():
    rclpy.init()
    node = HomicidalChauffeur()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()