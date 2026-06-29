"""OperatorCommandPort: driving port for operator to simulation commands.

The dashboard gains operator commands (mission start/stop, per-drone
return-home, investigate-here from the mission scene). Without this
port the Qt event handlers would call rclpy publishers directly,
fusing UI event logic to the ROS transport, the same
dependency-inversion problem the coordination layer solved with its
Clock / EventPort / TopicFactory ports.

Qt handlers depend on this Protocol; the ROS adapter (a thin wrapper
around publishers, living in the dashboard's node module) implements
it. Widget unit tests inject ``RecordingCommandAdapter`` and assert
on ``commands``; ``NullCommandAdapter`` is the default no-op for
surfaces constructed without a command path.

The method set deliberately mirrors the operator console actions and
nothing more (avoid speculative generality): survey start/stop ride
the existing ``/survey/start`` Bool topic; return-home rides the
lifecycle_manager's per-drone return topic; investigate publishes an
operator goal the mission_manager injects as an INVESTIGATE task.
"""

from __future__ import annotations

from typing import List, Protocol, Tuple


class OperatorCommandPort(Protocol):
    """Operator → simulation command contract."""

    def request_survey_start(self) -> None:
        """Ask the fleet to begin the survey mission."""
        ...

    def request_survey_stop(self) -> None:
        """Ask the fleet to stop surveying (drones return and land)."""
        ...

    def request_return_home(self, drone: str) -> None:
        """Ask one drone to abandon its task and return to base."""
        ...

    def request_investigate(self, x: float, y: float) -> None:
        """Ask the mission to investigate world position ``(x, y)``."""
        ...


class NullCommandAdapter:
    """No-op adapter for tests and read-only dashboard surfaces."""

    __slots__ = ()

    def request_survey_start(self) -> None:
        pass

    def request_survey_stop(self) -> None:
        pass

    def request_return_home(self, drone: str) -> None:
        pass

    def request_investigate(self, x: float, y: float) -> None:
        pass


class RecordingCommandAdapter:
    """Test adapter: records every command as a tuple, in order."""

    __slots__ = ('commands',)

    def __init__(self) -> None:
        self.commands: List[Tuple] = []

    def request_survey_start(self) -> None:
        self.commands.append(('survey_start',))

    def request_survey_stop(self) -> None:
        self.commands.append(('survey_stop',))

    def request_return_home(self, drone: str) -> None:
        self.commands.append(('return_home', drone))

    def request_investigate(self, x: float, y: float) -> None:
        self.commands.append(('investigate', x, y))
