import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints


def make_waypoints(node: Node):
    now = node.get_clock().now().to_msg()
    waypoints = []

    def euler_to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
        import math
        from geometry_msgs.msg import Quaternion
        q = Quaternion()
        q.x = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        q.y = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        q.z = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        q.w = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return q

    def pose(x, y, yaw):
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = euler_to_quaternion(0.0, 0.0, float(yaw))
        return pose


    waypoints.append(pose(1.0, 0.0, 0.0))
    waypoints.append(pose(0.0, 1.0, 0.0))
    waypoints.append(pose(0.0, 0.0, 0.0))
    return waypoints

class WaypointsFollower(Node):

    def __init__(self):
        super().__init__('waypoints_follower')
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

    def send_goal(self):
        # タイムアウト付きで待機
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("follow_waypoints action server not available.")
            rclpy.shutdown()
            return

        goal_msg = FollowWaypoints.Goal()
        waypoints = make_waypoints(self)
        goal_msg.poses = waypoints
        
        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
        return future

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'Current waypoint: {fb.current_waypoint}')

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal result: {result}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    action_client = WaypointsFollower()

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
