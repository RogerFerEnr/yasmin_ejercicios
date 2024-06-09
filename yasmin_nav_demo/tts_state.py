

from yasmin import Blackboard
from yasmin_ros import ActionState
from simple_node import Node
from audio_common_msgs.action import TTS


class TtsState(ActionState):
    def __init__(self, node: Node) -> None:
        super().__init__(
            node,  # node
            TTS,  # action type
            "/say",  # action name
            self.create_goal_handler,  # cb to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            None  # cb to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> TTS.Goal:

        goal = TTS.Goal()
        goal.text = blackboard.text
        return goal
