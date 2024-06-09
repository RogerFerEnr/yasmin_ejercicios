
import random
from yasmin import State
from outcomes import END
from yasmin.blackboard import Blackboard


class CreateWaypointListState(State):

    def __init__(self) -> None:
        super().__init__([END])

    def execute(self, blackboard: Blackboard) -> str:

        blackboard.waypoints_names = random.sample(
            list(blackboard.waypoints.keys()), 3
        )

        return END
