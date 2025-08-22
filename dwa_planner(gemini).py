import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import math
from enum import Enum


class RobotType(Enum):
    circle = 0
    rectangle = 1

class Config:
    def __init__(self):
        self.max_speed = 0.22
        self.min_speed = 0.0
        self.max_yaw_rate = 2.84
        self.max_accel = 0.2
        self.max_delta_yaw_rate = 3.2
        self.v_resolution = 0.01
        self.yaw_rate_resolution = 0.1
        self.dt = 0.1
        self.predict_time = 1.0
        self.to_goal_cost_gain = 1.0
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_radius = 0.2
        self.robot_stuck_flag_cons = 0.001
        self.robot_type = RobotType.circle

        self.goal_tolerance = 0.5  # meters
        


class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')
        self.config = Config()

        self.pose = [0.0, 0.0, 0.0]  # x, y, yaw
        self.velocity = [0.0, 0.0]  # v, omega
        self.goal = None
        self.scan = []
        self.goal_reached_flag = False

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg):
        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = self.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        self.pose[2] = yaw
        self.velocity[0] = msg.twist.twist.linear.x
        self.velocity[1] = msg.twist.twist.angular.z

    def euler_from_quaternion(self, quat):
        x, y, z, w = quat
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return 0, 0, math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        self.scan = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def goal_callback(self, msg):
        self.goal = [msg.pose.position.x, msg.pose.position.y]
        self.get_logger().info("Received goal: (%.2f, %.2f)" % (self.goal[0], self.goal[1]))
        self.goal_reached_flag = True

    def timer_callback(self):
        if not self.scan or not self.goal:
            return

        state = [self.pose[0], self.pose[1], self.pose[2], self.velocity[0], self.velocity[1]]
        dw = self.calc_dynamic_window(state)
        u, traj = self.calc_control_and_trajectory(state, dw)

        dx = self.goal[0] - self.pose[0]
        dy = self.goal[1] - self.pose[1]
        dist_to_goal = math.hypot(dx, dy)

        if dist_to_goal < self.config.goal_tolerance:

            if self.goal_reached_flag == True:
                self.get_logger().info("Goal reached!")
                self.goal_reached_flag = False

            cmd = Twist()  # stop the robot
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        cmd = Twist()
        cmd.linear.x = float(u[0])
        cmd.angular.z = float(u[1])
        self.cmd_pub.publish(cmd)
        self.publish_trajectories(traj)

    def motion(self, x, u):
        x[2] += u[1] * self.config.dt
        x[0] += u[0] * math.cos(x[2]) * self.config.dt
        x[1] += u[0] * math.sin(x[2]) * self.config.dt
        x[3] = u[0]
        x[4] = u[1]
        return x

    def calc_dynamic_window(self, x):
        Vs = [self.config.min_speed, self.config.max_speed,
              -self.config.max_yaw_rate, self.config.max_yaw_rate]
        Vd = [x[3] - self.config.max_accel * self.config.dt,
              x[3] + self.config.max_accel * self.config.dt,
              x[4] - self.config.max_delta_yaw_rate * self.config.dt,
              x[4] + self.config.max_delta_yaw_rate * self.config.dt]
        return [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
                max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    def predict_trajectory(self, x_init, v, w):
        x = x_init[:]
        traj = [x[:]]
        time = 0.0
        while time <= self.config.predict_time:
            x = self.motion(x, [v, w])
            traj.append(x[:])
            time += self.config.dt
        return traj

    def calc_control_and_trajectory(self, x, dw):
        min_cost = float('inf')
        best_u = [0.0, 0.0]
        best_traj = [x[:]]
        for v in np.arange(dw[0], dw[1], self.config.v_resolution):
            for w in np.arange(dw[2], dw[3], self.config.yaw_rate_resolution):
                traj = self.predict_trajectory(x[:], v, w)
                to_goal_cost = self.config.to_goal_cost_gain * self.calc_to_goal_cost(traj)
                speed_cost = self.config.speed_cost_gain * (self.config.max_speed - traj[-1][3])
                ob_cost = self.config.obstacle_cost_gain * self.calc_obstacle_cost(traj)
                final_cost = to_goal_cost + speed_cost + ob_cost
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, w]
                    best_traj = traj[:]
        return best_u, best_traj

    def calc_to_goal_cost(self, traj):
        dx = self.goal[0] - traj[-1][0]
        dy = self.goal[1] - traj[-1][1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - traj[-1][2]
        return abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    def calc_obstacle_cost(self, traj):
        if not self.scan:
            return 0.0
        min_dist = float('inf')
        for x in traj:
            for i, r in enumerate(self.scan):
                if math.isinf(r) or math.isnan(r):
                    continue
                angle = self.angle_min + i * self.angle_increment
                obs_x = self.pose[0] + r * math.cos(self.pose[2] + angle)
                obs_y = self.pose[1] + r * math.sin(self.pose[2] + angle)
                dist = math.hypot(x[0] - obs_x, x[1] - obs_y)
                if dist < min_dist:
                    min_dist = dist
        if min_dist == float('inf'):
            return 0.0
        if min_dist <= self.config.robot_radius:
            return float('inf')
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
        for x in traj:
            p = Point()
            p.x = x[0]
            p.y = x[1]
            p.z = 0.0
            marker.points.append(p)
        marker.id = 0
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)




    def calc_obstacle_cost(self, traj):
        if not self.scan:
            return 0.0
        min_dist = float('inf')
        for x in traj:
            for i, r in enumerate(self.scan):
                if math.isinf(r) or math.isnan(r):
                    continue
                angle = self.angle_min + i * self.angle_increment
                obs_x = self.pose[0] + r * math.cos(self.pose[2] + angle)
                obs_y = self.pose[1] + r * math.sin(self.pose[2] + angle)
                dist = math.hypot(x[0] - obs_x, x[1] - obs_y)
                if dist < min_dist:
                    min_dist = dist

        if min_dist == float('inf'):
            return 0.0

        # 🚨 New safety buffer
        safety_distance = 0.27 + self.config.robot_radius

        # If too close -> treat as collision
        if min_dist <= safety_distance:
            return float('inf')

        # Otherwise, cost inversely proportional to distance
        return 1.0 / min_dist


def main(args=None):
    rclpy.init(args=args)
    node = DWAPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
