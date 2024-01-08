#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
import math
from rclpy.qos import QoSProfile
from rclpy.node import Node

class Bug0(Node):
    def __init__(self):
        super().__init__('bug0')

        self.marker_pos = [(6.0, 2.0), (7.0, -5.0), (-3, -8.0), (-7.0, -1.5)]
        self.counter = 0

        self.active = False
        self.yaw_error_allowed = 5 * (math.pi / 180)  # 5 degrees
        self.position = Point()
        self.pose = Pose()
        self.desired_position = Point()
        #self.desired_position.z = 0.0
        self.regions_ = None
        self.state_desc = ['Go to point', 'wall following', 'done']
        self.state = 0
        # 0 - go to point
        # 1 - wall following
        # 2 - done
        # 3 - canceled

        self.srv_client_go_to_point = self.create_client(SetBool, '/go_to_point_switch')
        self.srv_client_wall_follower = self.create_client(SetBool, '/wall_follower_switch')
        self.client = self.create_client(SetBool, 'response_go_to')
        self.service = self.create_service(SetBool, 'go_to_spot', self.service_callback)

        while not (self.srv_client_go_to_point.wait_for_service(timeout_sec=1.0) and
                   self.srv_client_wall_follower.wait_for_service(timeout_sec=1.0) and
                   self.client.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info('Services not available, waiting again...')

        self.sub_laser = self.create_subscription(LaserScan, '/laser/scan', self.clbk_laser, QoSProfile(depth=10))
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.clbk_odom, QoSProfile(depth=10))
        self.pub = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=10))
        #self.desired_position.x = 0.0
        #self.desired_position.y = 0.0
        self.get_logger().info('Bug0 node initialized')

        self.timer = self.create_timer(0.1, self.run)

    def clbk_odom(self, msg):
        self.position = msg.pose.pose.position
        self.pose = msg.pose.pose
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.yaw = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        if self.yaw < 0:
            self.yaw = math.pi + (math.pi + self.yaw)

    def clbk_laser(self, msg):
        self.regions_ = {
            'right': min(min(msg.ranges[180:251]), 10),
            'fright': min(min(msg.ranges[252:323]), 10),
            'front': min(min(msg.ranges[324:395]), 10),
            'fleft': min(min(msg.ranges[396:467]), 10),
            'left': min(min(msg.ranges[468:539]), 10),
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
        self.active = request.data
        position = self.marker_pos[self.counter]
        self.counter += 1
        x_des, y_des = position
        self.desired_position.x = x_des
        self.desired_position.y = y_des
        response.success = True
        return response
    
    def run(self):
        if self.active:
            err_pos = math.sqrt(pow(self.desired_position.y - self.position.y, 2) +
                                pow(self.desired_position.x - self.position.x, 2))
            
            if err_pos < 0.5:
                self.change_state(2)
                self.done()
                self.end_iteration()

            elif self.regions_ is None:
                self.get_logger().info(" Waiting Laser information..")

            elif self.state == 0:
                if self.regions_['front'] < 1.0:
                    self.change_state(1)

            elif self.state == 1:
                desired_yaw = math.atan2(
                    self.desired_position.y - self.position.y, self.desired_position.x - self.position.x)
                err_yaw = self.normalize_angle(desired_yaw - self.yaw)

                if self.regions_['front'] > 1 and math.fabs(err_yaw) < 0.05:
                    self.change_state(0)

            elif self.state == 2:
                self.end_iteration()

            else:
                self.get_logger().error('Unknown state!')
    
    def end_iteration(self):
        self.active = False
        request = SetBool.Request()
        request.data = True
        future = self.client.call_async(request)
        future.add_done_callback(self.callback)


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

