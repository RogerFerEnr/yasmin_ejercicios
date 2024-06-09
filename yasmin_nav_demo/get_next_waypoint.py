
from yasmin import State
from outcomes import HAS_NEXT, END
from yasmin.blackboard import Blackboard
from geometry_msgs.msg import Pose


class GetNextWaypointState(State):

    def __init__(self) -> None:
        super().__init__([HAS_NEXT, END])

    def execute(self, blackboard: Blackboard) -> str:

        if not blackboard.waypoints_names:
            return END

        wp_name = blackboard.waypoints_names.pop(0)
        wp = blackboard.waypoints[wp_name]

        pose = Pose()
        pose.position.x = wp[0]
        pose.position.y = wp[1]
        pose.orientation.z = wp[2]
        pose.orientation.w = wp[3]

        blackboard.pose = pose
        blackboard.text = f"I have reached waypoint {wp_name}"

        return HAS_NEXT
