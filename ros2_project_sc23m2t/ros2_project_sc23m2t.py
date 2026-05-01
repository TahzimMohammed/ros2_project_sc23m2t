import threading
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
from action_msgs.msg import GoalStatus
import signal


class RobotProject(Node):
    def __init__(self):
        super().__init__('ros2_project_sc23m2t')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.bridge = CvBridge()
        self.sensitivity = 15

        self.red_detected   = False
        self.green_detected = False
        self.blue_detected  = False
        self.blue_close     = False

        self.detect_threshold = 300
        self.close_threshold  = 6000

        self.navigating       = False
        self.current_goal_idx = 0
        self.task_complete    = False

        self.waypoints = [
            (-1.0, -5.0,  270.0),
            (-2.0, -7.0,  270.0),
            (-3.5, -9.0,    0.0),
            ( 0.0, -7.0,    0.0),
            ( 3.0, -7.0,    0.0),
            ( 4.9, -7.5,   90.0),
            ( 5.0, -4.0,   90.0),
            ( 6.5, -2.0,   90.0),
            ( 6.9, -0.1,  180.0),
            ( 3.0, -4.0,  180.0),
            ( 0.0,  0.0,  180.0),
        ]

        self.get_logger().info('RobotProject node started.')

    def camera_callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        cv2.namedWindow('Camera Feed', cv2.WINDOW_NORMAL)
        cv2.imshow('Camera Feed', image)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        hsv_red_lower1 = np.array([0,   100, 100])
        hsv_red_upper1 = np.array([10,  255, 255])
        hsv_red_lower2 = np.array([170, 100, 100])
        hsv_red_upper2 = np.array([179, 255, 255])
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        hsv_blue_lower  = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper  = np.array([120 + self.sensitivity, 255, 255])

        mask_red   = cv2.bitwise_or(
                        cv2.inRange(hsv, hsv_red_lower1, hsv_red_upper1),
                        cv2.inRange(hsv, hsv_red_lower2, hsv_red_upper2))
        mask_green = cv2.inRange(hsv, hsv_green_lower, hsv_green_upper)
        mask_blue  = cv2.inRange(hsv, hsv_blue_lower,  hsv_blue_upper)

        mask_all = cv2.bitwise_or(cv2.bitwise_or(mask_red, mask_green), mask_blue)
        filtered = cv2.bitwise_and(image, image, mask=mask_all)
        cv2.namedWindow('Colour Detection', cv2.WINDOW_NORMAL)
        cv2.imshow('Colour Detection', filtered)

        image = self._detect_colour(image, mask_red,   'RED',   (0, 0, 255))
        image = self._detect_colour(image, mask_green, 'GREEN', (0, 255, 0))
        image = self._detect_blue(image,  mask_blue)

        cv2.namedWindow('Annotated Feed', cv2.WINDOW_NORMAL)
        cv2.imshow('Annotated Feed', image)
        cv2.waitKey(3)

    def _detect_colour(self, image, mask, colour_name, bgr):
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) > self.detect_threshold:
                (x, y), radius = cv2.minEnclosingCircle(c)
                cv2.circle(image, (int(x), int(y)), int(radius), bgr, 3)
                cv2.putText(image, colour_name,
                            (int(x) - 30, int(y) - int(radius) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, bgr, 2)
                if colour_name == 'RED' and not self.red_detected:
                    self.get_logger().info('RED box detected!')
                    self.red_detected = True
                elif colour_name == 'GREEN' and not self.green_detected:
                    self.get_logger().info('GREEN box detected!')
                    self.green_detected = True
        return image

    def _detect_blue(self, image, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > self.detect_threshold:
                (x, y), radius = cv2.minEnclosingCircle(c)
                cv2.circle(image, (int(x), int(y)), int(radius), (255, 0, 0), 3)
                cv2.putText(image, 'BLUE',
                            (int(x) - 30, int(y) - int(radius) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                if not self.blue_detected:
                    self.get_logger().info('BLUE box detected!')
                self.blue_detected = True
                if area > self.close_threshold:
                    self.blue_close = True
                    cv2.putText(image, 'WITHIN 1m - STOPPING',
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                0.7, (255, 0, 0), 2)
        return image

    def send_nav_goal(self, x, y, yaw_deg):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 action server not available!')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        yaw_rad = np.deg2rad(yaw_deg)
        goal_msg.pose.pose.orientation.z = float(np.sin(yaw_rad / 2.0))
        goal_msg.pose.pose.orientation.w = float(np.cos(yaw_rad / 2.0))

        self.get_logger().info(f'Navigating to waypoint ({x}, {y}, {yaw_deg})')
        self.navigating = True
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)
        return True

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            self.navigating = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Waypoint reached!')
        else:
            self.get_logger().warn(f'Navigation ended with status: {status}')
        self.navigating = False

    def stop(self):
        twist = Twist()
        self.publisher.publish(twist)
        self.get_logger().info('Robot stopped.')
        
    def cancel_goal(self):
        try:
            if self.nav_client._goal_handles:
                for goal_handle_ref in self.nav_client._goal_handles.values():
                    goal_handle = goal_handle_ref()
                    if goal_handle is not None:
                        goal_handle.cancel_goal_async()
        except Exception as e:
            self.get_logger().warn(f'Could not cancel goal: {e}')


def main():
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    rclpy.init(args=None)
    robot = RobotProject()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    time.sleep(2.0)

    try:
        while rclpy.ok():

            if robot.blue_close and robot.red_detected and robot.green_detected:
                robot.get_logger().info(
                    f'Task complete! Red: {robot.red_detected}, '
                    f'Green: {robot.green_detected}, Blue: {robot.blue_detected}')
                robot.cancel_goal()
                robot.stop()
                robot.task_complete = True
                break

            if robot.navigating:
                time.sleep(0.2)
                continue

            if robot.current_goal_idx < len(robot.waypoints):
                wp = robot.waypoints[robot.current_goal_idx]
                robot.send_nav_goal(wp[0], wp[1], wp[2])
                robot.current_goal_idx += 1
            else:
                robot.get_logger().info('All waypoints visited, restarting.')
                robot.current_goal_idx = 0

            time.sleep(0.2)

    except ROSInterruptException:
        pass

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()