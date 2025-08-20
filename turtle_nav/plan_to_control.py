import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

class PlanToControllerNode(Node):
    def __init__(self):
        super().__init__('plan_to_controller_node')

        # Subscribe to the /plan topic
        self.subscription = self.create_subscription(
            Path,
            '/robot2/plan',
            self.plan_callback,
            qos_profile_sensor_data
        )

        # Action client for /follow_path
        self._action_client = ActionClient(self, FollowPath, '/robot2/follow_path')

        self.get_logger().info('Node initialized and listening to /plan')

    def plan_callback(self, path_msg):
        if not path_msg.poses:
            self.get_logger().warn('Received empty path. Skipping...')
            return

        self.get_logger().info(f'Received path with {len(path_msg.poses)} poses. Sending to controller...')

        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg

        # Wait for the action server to be available
        self._action_client.wait_for_server()

        # Send the goal
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal was rejected!')
            return

        self.get_logger().info('FollowPath goal accepted. Waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('FollowPath action finished with result code: {}'.format(result))


def main():
    rclpy.init()
    node = PlanToControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()