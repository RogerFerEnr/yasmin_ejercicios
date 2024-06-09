# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import unittest
import time
from threading import Thread

from yasmin_ros import ActionState, ServiceState
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT

from example_interfaces.action import Fibonacci
from example_interfaces.srv import AddTwoInts

from simple_node import Node
import rclpy


class AuxNode(Node):
    def __init__(self):
        super().__init__("test_node")

        self.action_server = self.create_action_server(
            Fibonacci, "test", self.execute_action)

        self.service_server = self.create_service(
            AddTwoInts, "test", self.execute_service)

    def execute_action(self, goal_handle):
        result = Fibonacci.Result()

        request = goal_handle.request

        if request.order >= 0:

            counter = 0
            while not goal_handle.is_cancel_requested and counter < request.order:
                time.sleep(1)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

            else:
                goal_handle.succeed()

        else:
            goal_handle.abort()

        return result

    def execute_service(self, request, response):
        response.sum = request.a + request.b
        return response


class TestYasminRos(unittest.TestCase):
    def setUp(self):
        super().setUp()

        rclpy.init()
        self.node = Node("test_yasmin_node")
        self.aux_node = AuxNode()

        # states

    def tearDown(self):
        super().tearDown()
        rclpy.shutdown()

    def test_yasmin_ros_action_succeed(self):

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 0
            return goal

        state = ActionState(self.node, Fibonacci, "test", create_goal_cb)
        self.assertEqual(SUCCEED, state())

    def test_yasmin_ros_action_result_handler(self):

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 0
            return goal

        def result_handler(blackboard, result):
            return "new_outcome"

        state = ActionState(self.node, Fibonacci, "test", create_goal_cb, [
            "new_outcome"], result_handler)
        self.assertEqual("new_outcome", state())

    def test_yasmin_ros_action_cancel(self):

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = 20
            return goal

        def cancel_state(state, seconds):
            time.sleep(seconds)
            state.cancel_state()

        state = ActionState(self.node, Fibonacci, "test", create_goal_cb)
        thread = Thread(target=cancel_state, args=(state, 5,))
        thread.start()
        self.assertEqual(CANCEL, state())
        thread.join()

    def test_yasmin_ros_action_abort(self):

        def create_goal_cb(blackboard):
            goal = Fibonacci.Goal()
            goal.order = -1
            return goal

        state = ActionState(self.node, Fibonacci, "test", create_goal_cb)
        self.assertEqual(ABORT, state())

    def test_yasmin_ros_service(self):

        def create_request_cb(blackboard):
            request = AddTwoInts.Request()
            request.a = 2
            request.b = 3
            return request

        state = ServiceState(self.node, AddTwoInts, "test", create_request_cb)
        self.assertEqual(SUCCEED, state())

    def test_yasmin_ros_service_response_handler(self):

        def create_request_cb(blackboard):
            request = AddTwoInts.Request()
            request.a = 2
            request.b = 3
            return request

        def response_handler(blackboard, response):
            return "new_outcome"

        state = ServiceState(self.node, AddTwoInts, "test",
                             create_request_cb, ["new_outcome"], response_handler)
        self.assertEqual("new_outcome", state())
