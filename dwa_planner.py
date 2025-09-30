import rclpy
from rclpy.node import Node

import math
import numpy as np
import random
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import euler_from_quaternion

from std_msgs.msg import String


class Minimal(Node):

    def __init__(self):
        super().__init__('dwa_planner')

        self.flag = False # for testing
        self.pose = [0.0,0.0,0.0]
        self.velocity = [0.0,0.0]
        self.state = []

        self.scan = []
        self.angle_min = 0.0
        self.angle_increment = 0.0
        self.minimum_scan_range = 0.2
        self.maximum_scan_range = 4.0

        self.goal = []
        self.goal_x,self.goal_y = 0.0, 0.0
        self.goal_reached_flag = True 
        self.goal_tolerance = 0.5       

        self.max_speed = 0.2
        self.min_speed = -0.1
        self.max_omega = 40 * math.pi/180
        self.max_lin_accel = 0.1
        self.max_omeg_accel = 40 * math.pi/180.0

        self.dt = 0.4 # earlier 0.5
        self.v_resolution = 0.08 # earlier 0.01
        self.omega_resolution =  0.4 * math.pi/180.0 # earlier 0.1 * math.pi/180.0
        self.predict_time = 4.0 # earlier 2.0
        self.robot_radius = 0.22

        self.to_goal_cost_gain = 0.4
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 0.5
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 1)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 1)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.marker_pub = self.create_publisher(Marker, '/dwa_trajectories', 1)

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def odom_callback(self, msg):
        self.pose[0]= msg.pose.pose.position.x
        self.pose[1]= msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([orientation.x,orientation.y,orientation.z,orientation.w])
        self.pose[2]= yaw
        self.velocity[0] = msg.twist.twist.linear.x
        self.velocity[1] = msg.twist.twist.angular.z

    def scan_callback(self, msg):
        self.scan = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.minimum_scan_range = msg.range_min
        self.maximum_scan_range = msg.range_max

    def goal_callback(self, msg):
        self.goal = [msg.pose.position.x,msg.pose.position.y]

        self.get_logger().info('Goal recieved at x: "%s" and y: "%s"' %(round(self.goal[0],2),round(self.goal[1],2)))
        self.goal_reached_flag = False



    def timer_callback(self):
        if not self.scan or not self.goal:
            return


        goal = [self.goal[0],self.goal[1]]
        state = [self.pose[0],self.pose[1],self.pose[2],self.velocity[0],self.velocity[1]]
        scanned = [self.scan, self.angle_min, self.angle_increment, self.minimum_scan_range, self.maximum_scan_range]

        u, best_trajectory_to_draw = self.dwa_control(goal, state, scanned)  

        dx = goal[0] - state[0]
        dy = goal[1] - state[1]
        dist_to_goal = math.sqrt(dx**2 + dy**2)

        cmd = Twist()

        if dist_to_goal < self.goal_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info("Goal Reached")
            # self.get_logger().info(f"Publishing cmd_vel: linear.x={cmd.linear.x}, angular.z={cmd.angular.z}")

        else:
            cmd.linear.x = float(u[0])
            cmd.angular.z = float(u[1])
            self.cmd_pub.publish(cmd)
            # self.get_logger().info(f"Publishing cmd_vel: linear.x={cmd.linear.x}, angular.z={cmd.angular.z}")



    def dwa_control(self, goal, state, scanned):  

        v_min = state[3] - self.max_lin_accel * self.dt
        v_max = state[3] + self.max_lin_accel * self.dt
        omega_min = state[4] - self.max_omeg_accel * self.dt
        omega_max = state[4] + self.max_omeg_accel * self.dt

        dw = {
            "v_min":max(self.min_speed, v_min),
            "v_max":min(self.max_speed, v_max),
            "omega_min":max(-self.max_omega, omega_min),
            "omega_max":min(self.max_omega,omega_max),
        }

        best_cost = float("inf")
        best_u = [0.0,0.0]

        best_trajectory = np.array([[state[0],state[1]]])

        trajectory_visualization_xy = []

        for v in np.arange(dw["v_min"],dw["v_max"], self.v_resolution):
            for omega in np.arange(dw["omega_min"],dw["omega_max"],self.omega_resolution):
                trajectory = self.predict_trajectory(state, v, omega)

                to_goal_cost = self.to_goal_cost_gain * self.calc_to_goal_cost(trajectory, goal)
                
                speed_cost = self.speed_cost_gain * (self.max_speed - trajectory[-1,3])
                
                obstacle_cost = self.obstacle_cost_gain * self.calc_obstacle_cost(trajectory, state, scanned) # added scan in calc_obstacle_cost
                

                final_cost = to_goal_cost + speed_cost + obstacle_cost

       
                if final_cost < best_cost:
                    best_cost = final_cost
                    best_u = [v, omega]
                    best_trajectory = trajectory

            
        
        return best_u, best_trajectory

    def predict_trajectory(self, state, v, omega):
        current_state = np.array([state[0],state[1],state[2],state[3],state[4]])
        trajectory = np.array(current_state)
        time = 0

        while time <= self.predict_time:
            current_state[2] += omega*self.dt
            current_state[0] += v * math.cos(current_state[2]) * self.dt
            current_state[1] += v * math.sin(current_state[2]) * self.dt 
            current_state[3] = v
            current_state[4] = omega

            trajectory = np.vstack((trajectory, current_state))
            time += self.dt 

        return trajectory

    def calc_obstacle_cost(self, trajectory, state, scanned):
        min_dist = float("inf")
        obs_x = []
        obs_y = []
        obs_xy = []

        for i in range (len(scanned[0])):
            if scanned[3] < scanned[0][i] < scanned[4]:
                obs_angle = scanned[1] + i*scanned[2] + state[2]

                obstacle_x = scanned[0][i] * math.cos(obs_angle) + state[0] 
                obstacle_y = scanned[0][i] * math.sin(obs_angle) + state[1]
                obs_xy.append((obstacle_x,obstacle_y))

            elif math.isinf(scanned[0][i]):

                obs_xy.append((100,100))
            else:

                obs_xy.append((0.1,0.1))


                
        for i in range (len(trajectory)):
            for ox, oy in obs_xy:
                dist = math.sqrt((trajectory[i,0] - ox)**2 + (trajectory[i,1] - oy)**2)
                if dist <= self.robot_radius:
                    return float("inf")
                min_dist = min (dist, min_dist)





        return 1.0/min_dist

    def calc_to_goal_cost(self,trajectory,goal):
        # dx = goal[0] - trajectory[-1,0]
        # dy = goal[1] - trajectory[-1,1]
        # error_angle = math.atan2(dy,dx)
        # cost_angle = error_angle - trajectory[-1,2]

        # cost = abs(math.atan2(math.sin(cost_angle),math.cos(cost_angle)))
        # return cost
        dx =   goal[0] - trajectory[-1,0]
        dy =   goal[1] - trajectory[-1,1] 
        error_angle = math.atan2(dy,dx)
        cost_angle =   error_angle -  trajectory[-1,2]

        cost = abs(math.atan2(math.sin(cost_angle),math.cos(cost_angle)))
        return cost


def main(args=None):
    rclpy.init(args=args)

    dynamic_node = Minimal()

    rclpy.spin(dynamic_node)
    dynamic_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()