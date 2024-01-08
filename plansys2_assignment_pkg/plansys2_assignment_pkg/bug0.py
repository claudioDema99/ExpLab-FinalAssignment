#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
import math
import time
from rclpy.qos import QoSProfile
from rclpy.node import Node
from my_interface.action import MyRos2Plan
from action_msgs.msg import GoalStatus

class Bug0(Node):
    def __init__(self):
        super().__init__('bug0')

        self.yaw_error_allowed = 5 * (math.pi / 180)  # 5 degrees
        self.position = Point()
        self.pose = Pose()
        self.desired_position = Point()
        self.desired_position.z = 0.0
        self.regions = None
        self.state_desc = ['Go to point', 'wall following', 'done']
        self.state = 0
        # 0 - go to point
        # 1 - wall following
        # 2 - done
        # 3 - canceled

        self.srv_client_go_to_point = self.create_client(SetBool, '/go_to_point_switch')
        self.srv_client_wall_follower = self.create_client(SetBool, '/wall_follower_switch')
        self.client = self.create_client(SetBool, 'response_go_to')                                   # CLT FOR SEND THAT THE ACTION IS COMPLETED

        self.service = self.create_service(SetBool, 'go_to_marker', self.service_callback)            # SRV FOR STARTING THE ACTION

        while not (self.srv_client_go_to_point.wait_for_service(timeout_sec=1.0) and
                   self.srv_client_wall_follower.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info('Services not available, waiting again...')
        
        while not (self.client.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info('Services CLIENT not available, waiting again...')

        self.sub_laser = self.create_subscription(LaserScan, '/laser/scan', self.clbk_laser, QoSProfile(depth=10))
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.clbk_odom, QoSProfile(depth=10))
        self.pub = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=10))

        #self.act_s = self.create_server(Planning, '/reaching_goal', self.planning)
        #self.action_server = ActionServer(self, MyRos2Plan, 'my_ros2_plan', self.exe_callback)
        
        # initialize going to the point
        self.desired_position.x = 5.0
        self.desired_position.y = 5.0
        self.get_logger().info('Bug0 node initialized')

    def clbk_odom(self, msg):
        self.position = msg.pose.pose.position
        self.pose = msg.pose.pose
        self.get_logger().info(' RICEVO ODOM')
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.yaw = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        if self.yaw < 0:
            self.yaw = math.pi + (math.pi + self.yaw)

    def clbk_laser(self, msg):
        self.get_logger().info(' RICEVO LASER')
        self.regions = {
            'right': min(min(msg.ranges[100:139]), 10),
            'fright': min(min(msg.ranges[140:179]), 10),
            'front': min(min(msg.ranges[180:219]), 10),
            'fleft': min(min(msg.ranges[220:259]), 10),
            'left': min(min(msg.ranges[260:299]), 10),
        }

    def change_state(self, state):
        self.state = state
        log = f"state changed: {self.state_desc[state]}"
        self.get_logger().info(log)
        if self.state == 0:
            resp = self.srv_client_go_to_point.call_async(SetBool.Request(data=True))
            resp = self.srv_client_wall_follower.call_async(SetBool.Request(data=False))
        elif self.state == 1:
            resp = self.srv_client_go_to_point.call_async(SetBool.Request(data=False))
            resp = self.srv_client_wall_follower.call_async(SetBool.Request(data=True))
        elif self.state == 2:
            resp = self.srv_client_go_to_point.call_async(SetBool.Request(data=False))
            resp = self.srv_client_wall_follower.call_async(SetBool.Request(data=False))

    def normalize_angle(self, angle):
        if math.fabs(angle) > math.pi:
            angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
        return angle

    def done(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.pub.publish(twist_msg)

    def service_callback(self, request, response):
        self.change_state(0)
        rate = self.create_rate(20)
        #self.desired_position.x = goal_handle.request.x_goal
        #self.desired_position.y = goal_handle.request.y_goal
        i = 0

        while rclpy.ok():
            err_pos = math.sqrt(pow(self.desired_position.y - self.position.y, 2) +
                                pow(self.desired_position.x - self.position.x, 2))
            
            if err_pos < 0.5:
                self.change_state(2)
                self.done()
                break

            elif self.regions is None:
                continue

            elif self.state == 0:
                if self.regions['front'] < 0.2: # SE NON RICEVO LASER CALLBACK ERRORE
                    self.change_state(1)

            elif self.state == 1:
                desired_yaw = math.atan2(
                    self.desired_position.y - self.position.y, self.desired_position.x - self.position.x)
                err_yaw = self.normalize_angle(desired_yaw - self.yaw)

                if self.regions['front'] > 1 and math.fabs(err_yaw) < 0.05:
                    self.change_state(0)

            elif self.state == 2:
                break

            else:
                self.get_logger().error('Unknown state!')
            """            
            i += 1
            #time.sleep(0.1) METTENDO QUESTO NON SI STOPPA PIU
            #rate.sleep()
            if i == 100:
                self.change_state(2)
                self.done()
                break
            """

        request = SetBool.Request()
        request.data = True
        future = self.client.call_async(request)
        future.add_done_callback(self.callback)
        response.success = True
        return response

    def callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info('Service call failed: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    bug0_node = Bug0()
    rclpy.spin(bug0_node)
    bug0_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
