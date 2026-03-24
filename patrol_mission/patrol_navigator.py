import math
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Int32
from action_msgs.msg import GoalStatus

def euler_to_quaternion(roll=0.0, pitch=0.0, yaw=0.0) -> Quaternion:
    q = Quaternion()
    q.x = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    q.y = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    q.z = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    q.w = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return q


def make_pose(node: Node, x: float, y: float, yaw: float, frame_id: str = "map") -> PoseStamped:
    now = node.get_clock().now().to_msg()
    ps = PoseStamped()
    ps.header.stamp = now
    ps.header.frame_id = frame_id
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.orientation = euler_to_quaternion(0.0, 0.0, float(yaw))
    return ps


class PatrolNavigator(Node):
    def __init__(self):
        super().__init__("patrol_navigator")

        # (x, y, yaw)
        self.initial_pose = (0.0, 0.0, 0.0)
        self.waypoints = [
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 1.57),
            (0.0, 1.0, 3.14),
        ]
        # 最後に出発点に戻る
        self.waypoints.append(self.initial_pose)
        # ループするかどうか
        self.is_repeating = True
        # 現在の目標ウェイポイントのインデックス
        self.index = 0
        # 出発点のインデックス
        self.initial_pose_index = len(self.waypoints) - 1
        # バッテリ残量の閾値
        self.battery_threshold = 60
        # バッテリー残量が少ない場合に出発点に戻るためのモードのフラグ
        self.is_returning_to_initial_pose = False
        
        # 現在の目標ゴールハンドル
        self.current_goal_handle = None

        # ウェイポイントを送信するためのActionClient
        self.action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        # バッテリー状態を受信するサブスクライバ
        self.buttery_state_sub = self.create_subscription(
            Int32,
            "/battery_state",
            self.battery_state_callback,
            10,
        )

    def start(self):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose action server not available.")
            rclpy.shutdown()
            return

        self.send_next_goal()

    def send_next_goal(self):
        x, y, yaw = self.waypoints[self.index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = make_pose(self, x, y, yaw, frame_id="map")

        self.get_logger().info(f"Send waypoint[{self.index}] = ({x}, {y}, {yaw})")

        future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().debug(f"distance_remaining: {fb.distance_remaining:.3f}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self.current_goal_handle = goal_handle

        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected.")
            rclpy.shutdown()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status

        self.get_logger().info(f"Waypoint[{self.index}] done. status={status}, result={result}")

        # バッテリー残量が少ない場合は出発点に戻る
        if self.is_returning_to_initial_pose:
            if self.index == self.initial_pose_index and status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Arrived home. Shutdown.")
            else:
                self.get_logger().warn("Returning-home goal did not succeed (canceled/aborted). Shutdown anyway.")
            rclpy.shutdown()
            return

        # 次へウェイポイントを進める
        self.index += 1
        
        # すべてのウェイポイントを巡回したか確認
        if self.index >= len(self.waypoints):
            # リピートするなら最初のウェイポイントへ戻る
            if self.is_repeating:
                self.index = 0
                self.get_logger().info("Loop to waypoint[0]")
            # リピートしないなら終了
            else:
                self.get_logger().info("All waypoints finished.")
                rclpy.shutdown()
                return

        self.send_next_goal()

    def battery_state_callback(self, msg: Int32):
        # すでに出発点に戻るモードに入っているなら何もしない
        if self.is_returning_to_initial_pose:
            return
        
        battery_level = msg.data
        self.get_logger().info(f"Battery level: {battery_level}%")

        if battery_level <= self.battery_threshold:
            self.get_logger().warn("Battery low! Switching to return-home mode.")
            # 出発点に帰るフラグを立てる
            self.is_returning_to_initial_pose = True
            # 出発点のインデックスに切り替える
            self.is_repeating = False
            # ゴールのインデックスを出発点に切り替える
            self.index = self.initial_pose_index

            # 現在のゴールをキャンセルする
            self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNavigator()
    node.start()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
