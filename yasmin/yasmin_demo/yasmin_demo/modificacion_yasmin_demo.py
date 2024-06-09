#!/usr/bin/env python3

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


import time
import rclpy

from simple_node import Node

from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_viewer import YasminViewerPub


# define state Foo
class FooState(State):
    def __init__(self) -> None:
        super().__init__(["outcome1", "outcome2"])
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        print("Executing state FOO")
        time.sleep(3)

        if self.counter >0:
            print(blackboard.bar_str)

        if self.counter < 3:
            self.counter += 1
            blackboard.foo_str = f"Counter: {self.counter}"
            return "outcome1"
        else:
            return "outcome2"


# define state Bar
class BarState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome3","outcome4"])
        self.counter = 0
        self.iteraciones = 0
        self.blackboard = ""

    def execute(self, blackboard: Blackboard) -> str:
        print("Executing state BAR")
        time.sleep(3)

        if self.blackboard == "" or self.blackboard != blackboard.foo_str:
            self.counter=0
            self.blackboard = blackboard.foo_str
        
        self.iteraciones = self.iteraciones + 1 

        blackboard.bar_str = f"Se ha ejecutado {self.iteraciones} veces el estado BAR"
        
        if self.counter <2:
            self.counter = self.counter +1
            #print(blackboard.foo_str)
            return "outcome4"    
        else:
            print(blackboard.foo_str)
            return "outcome3"


# define state Nuevo
class NuevoState(State):
    def __init__(self) -> None:
        super().__init__(outcomes=["outcome5"])

    def execute(self, blackboard: Blackboard) -> str:
        print("Executing state NUEVO")
        time.sleep(3)

        #print(blackboard.foo_str)
        return "outcome5"





class DemoNode(Node):

    def __init__(self) -> None:
        super().__init__("yasmin_node")

        # create a state machine
        sm = StateMachine(outcomes=["outcome6"])

        # add states
        sm.add_state("FOO", FooState(),
                     transitions={"outcome1": "BAR",
                                  "outcome2": "outcome6"})
        sm.add_state("BAR", BarState(),
                     transitions={"outcome3": "FOO",
                                  "outcome4": "NUEVO"})
        sm.add_state("NUEVO",NuevoState(),
                     transitions={"outcome5": "BAR"})

        # pub
        YasminViewerPub(self, "YASMIN_DEMO_NUEVO", sm)

        # execute
        outcome = sm()
        print(outcome)


# main
def main(args=None):

    print("yasmin con estado NUEVO añadido")
    rclpy.init(args=args)
    node = DemoNode()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
