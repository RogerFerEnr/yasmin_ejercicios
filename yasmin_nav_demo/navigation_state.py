

from yasmin import Blackboard
from yasmin_ros import ActionState
from simple_node import Node
from nav2_msgs.action import NavigateToPose


class NavigationState(ActionState):
    def __init__(self, node: Node) -> None:
        super().__init__(
            node,  # node
            NavigateToPose,  # action type
            "/navigate_to_pose",  # action name
            self.create_goal_handler,  # cb to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            None  # cb to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> NavigateToPose.Goal:

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose = blackboard.pose
        return goal
