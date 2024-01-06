#!/usr/bin/python3

# Copyright 2023 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import time
from rclpy.parameter import Parameter, ParameterType
from rclpy.action import ActionClient
from rclpy.node import Node
from my_interface.action import MyRos2Plan
from action_msgs.msg import GoalStatus
from plansys2_support_py.ActionExecutorClient import ActionExecutorClient
from std_srvs.srv import SetBool

class GoToAction(ActionExecutorClient):

    def __init__(self):
        super().__init__('go_to_marker', 0.5)
        #self.action_client = ActionClient(self, MyRos2Plan, 'my_ros2_plan')
        self.client = self.create_client(SetBool, 'go_to_marker')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.service = self.create_service(SetBool, 'response_go_to', self.service_callback)


    def callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info('Service call failed: %s' % str(e))
    
    def service_callback(self, request, response):
        self.finish(True, 1.0, 'Charge completed')
        response.success = True
        return response

    def do_work(self):
        request = SetBool.Request()
        request.data = True
        future = self.client.call_async(request)
        future.add_done_callback(self.callback)

def main(args=None):
    rclpy.init(args=args)
    node = GoToAction()
    node.set_parameters([Parameter(name='action_name', value='go_to_marker')])
    node.trigger_configure()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

