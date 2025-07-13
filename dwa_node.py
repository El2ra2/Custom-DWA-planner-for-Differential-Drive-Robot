import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import math
import tf_transformations

class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')
        self.declare_parameter('goal_threshold', 0.3)
        self.goal_x = None
        self.goal_y = None
        self.goal_threshold = self.get_parameter('goal_threshold').value

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.pose = [0.0, 0.0, 0.0]  # x, y, theta
        self.velocity = [0.0, 0.0]   # linear, angular
        self.scan = []

        # Robot config
        self.max_speed = 0.2
        self.min_speed = 0.0
        self.max_yaw_rate = 2.8
        self.max_accel = 0.2
        self.max_delta_yaw_rate = 3.2
        self.v_reso = 0.01
        self.yaw_rate_reso = 0.1
        self.dt = 0.1
        self.predict_time = 1.0
        self.robot_radius = 0.2

    def odom_callback(self, msg):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        ignore,ignore , yaw = tf_transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        self.pose[2] = yaw
        self.velocity[0] = msg.twist.twist.linear.x
        self.velocity[1] = msg.twist.twist.angular.z

    def scan_callback(self, msg):
        self.scan = msg.ranges
        self.scan_angle_min = msg.angle_min
        self.scan_angle_increment = msg.angle_increment

    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.get_logger().info("Received new goal: (%.2f, %.2f)" % (self.goal_x, self.goal_y))


    def timer_callback(self):
        if not self.scan or self.goal_x is None or self.goal_y is None:
            return

        distance_to_goal = math.hypot(self.goal_x - self.pose[0], self.goal_y - self.pose[1])
        if distance_to_goal < self.goal_threshold:
            twist = Twist()
            self.cmd_pub.publish(twist)  # Stop the robot
            self.get_logger().info("Goal reached. Stopping robot.")
            return

        best_u, best_traj = self.dwa_control()

        twist = Twist()
        twist.linear.x = best_u[0]
        twist.angular.z = best_u[1]
        self.cmd_pub.publish(twist)

        self.publish_trajectories(best_traj)

    def dwa_control(self):
        min_v = max(self.min_speed, self.velocity[0] - self.max_accel * self.dt)
        max_v = min(self.max_speed, self.velocity[0] + self.max_accel * self.dt)
        min_w = self.velocity[1] - self.max_delta_yaw_rate * self.dt
        max_w = self.velocity[1] + self.max_delta_yaw_rate * self.dt

        best_u = [0.0, 0.0]
        best_traj = []
        min_cost = float('inf')

        for v in np.arange(min_v, max_v, self.v_reso):
            for w in np.arange(min_w, max_w, self.yaw_rate_reso):
                traj = self.predict_trajectory(v, w)
                to_goal_cost = self.calc_to_goal_cost(traj)
                obstacle_cost = self.calc_obstacle_cost(traj)
                speed_cost = self.max_speed - v

                final_cost = 1.0 * to_goal_cost + 1.5 * obstacle_cost + 0.5 * speed_cost

                if final_cost < min_cost:
                    min_cost = final_cost
                    best_u = [v, w]
                    best_traj = traj

        return best_u, best_traj

    def predict_trajectory(self, v, w):
        traj = []
        x = self.pose[0]
        y = self.pose[1]
        yaw = self.pose[2]
        time = 0.0
        while time <= self.predict_time:
            x += v * math.cos(yaw) * self.dt
            y += v * math.sin(yaw) * self.dt
            yaw += w * self.dt
            traj.append((x, y))
            time += self.dt
        return traj

    def calc_to_goal_cost(self, traj):
        dx = self.goal_x - traj[-1][0]
        dy = self.goal_y - traj[-1][1]
        return math.hypot(dx, dy)

    def calc_obstacle_cost(self, traj):
        if not self.scan:
            return 0.0
        min_dist = float('inf')
        for (x, y) in traj:
            for i, r in enumerate(self.scan):
                if math.isinf(r) or math.isnan(r):
                    continue
                angle = self.scan_angle_min + i * self.scan_angle_increment
                obs_x = self.pose[0] + r * math.cos(self.pose[2] + angle)
                obs_y = self.pose[1] + r * math.sin(self.pose[2] + angle)
                dist = math.hypot(x - obs_x, y - obs_y)
                if dist < min_dist:
                    min_dist = dist
        if min_dist == float('inf'):
            return 0.0
        return 1.0 / min_dist

    def publish_trajectories(self, traj):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0

        for (x, y) in traj:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        marker.id = 0
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = DWAPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
