import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import time
import math

class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'robot1/navigate_to_pose')
        self.waypoints = self.create_waypoints()

    def create_waypoints(self):
        # Replace with your actual waypoints (x, y, yaw in radians)
        points = [
            (-1.1888759073286665, 0.9567395337275825, -0.9999065960634634, 0.013667448510159852),
            (-1.7573231267877698, 1.1763000837780282, 0.9846082376343565, 0.17477590904517323),
            (-2.0487147682992837, 0.6371434216059553, 0.9860494100601587, 0.16645287897784494),
            (-2.761993990854728, 0.7684627066245723, 0.9944861030575851, 0.10486844532717292),
            (-2.523020336341442, 1.5389058964368052, 0.9993407460587141, 0.03630527877337653),
            (-3.0333638445221593, 1.6243871917065487, 0.9954135661976796, 0.0956652090867823),
            (-3.575691666712014, 0.8540813600863703, 0.9981897488383554, 0.06014337298506664)
        ]

        poses = []
        for x, y, z, w in points:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = z
            pose.pose.orientation.w = w
            poses.append(pose)
        return poses

    def send_waypoints(self):
        for pose in self.waypoints:
            self.send_goal(pose)
            rclpy.spin_until_future_complete(self, self._goal_future)

            goal_handle = self._goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Goal rejected!')
                continue

            self.get_logger().info('Goal accepted, waiting for result...')
            self._result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, self._result_future)
            result = self._result_future.result().result

            self.get_logger().info('Result received!')
            time.sleep(1)


    def send_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self._action_client.wait_for_server()
        self._goal_future = self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    navigator.send_waypoints()
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
