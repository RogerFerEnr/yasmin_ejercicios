
import rclpy
from simple_node.node import Node

from yasmin import StateMachine
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import *
from yasmin_viewer import YasminViewerPub

from navigation_state import NavigationState
from tts_state import TtsState
from create_waypoint_list import CreateWaypointListState
from get_next_waypoint import GetNextWaypointState
from outcomes import *


class NavFsmNode(Node):

    def __init__(self):

        super().__init__("nav_fsm_node")

        fsm = StateMachine([SUCCEED, CANCEL, ABORT])
        nav_fsm = StateMachine([SUCCEED, CANCEL, ABORT])

        fsm.add_state(
            "CREATING_WAYPOINTS",
            CreateWaypointListState(),
            {END: "NAVIGATING_FSM"}
        )

        nav_fsm.add_state(
            "GETTING_NEXT_WAYPOINT",
            GetNextWaypointState(),
            {
                HAS_NEXT: "NAVIGATING",
                END: SUCCEED
            }
        )

        nav_fsm.add_state(
            "NAVIGATING",
            NavigationState(self),
            {
                SUCCEED: "SPEAKING",
                CANCEL: CANCEL,
                ABORT: ABORT
            }
        )

        nav_fsm.add_state(
            "SPEAKING",
            TtsState(self),
            {
                SUCCEED: "GETTING_NEXT_WAYPOINT",
                CANCEL: CANCEL,
                ABORT: ABORT
            }
        )

        fsm.add_state(
            "NAVIGATING_FSM",
            nav_fsm,
            {
                SUCCEED: SUCCEED,
                CANCEL: CANCEL,
                ABORT: ABORT
            }
        )

        YasminViewerPub(self, "YASMIN_NAV_DEMO", fsm)

        blackboard = Blackboard()
        blackboard.waypoints = {
            "entrance": [1.25, 6.30, -0.78, 0.67],
            # "bathroom": [5.0, 3.68, 0.81, 0.59],
            "livingroom": [1.55, 4.03, -0.69, 0.72],
            "kitchen": [3.79, 6.77, 0.99, 0.12],
            "bedroom": [7.50, 4.89, 0.76, 0.65],
        }
        fsm(blackboard)


def main():
    rclpy.init()
    node = NavFsmNode()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
