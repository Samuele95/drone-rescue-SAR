"""OperatorView Protocol: the operator-facing view contract.

Driving port for any operator-facing surface that projects a
``MissionViewModel`` snapshot into its native rendering medium. Today
the only conforming implementation is the dashboard's
``MissionSceneView`` (Qt-native top-down scene); the
``drone_rescue_viz/`` RViz overlay nodes remain on their own
data-stream shape and are *not* OperatorView implementations, owing to
the data-shape mismatch and the conditions under which they would
join the contract.

The Protocol is intentionally minimal: `render_from(view)` is the
one method the contract requires. Concrete implementations may carry
arbitrary internal state (Qt items, MarkerArray caches, trail
deques) but their refresh dispatcher must accept a
``MissionViewModel`` snapshot and produce the appropriate native
output.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Deque, Mapping, Protocol, Tuple

if TYPE_CHECKING:
    from .view_model import MissionViewModel


class OperatorView(Protocol):
    """Operator-facing view contract.

    Concrete implementations:
    - ``drone_rescue_dashboard.scene_view.MissionSceneView``: Qt-native
      top-down scene with QGraphicsScene cursors + trail paths.
    - (deferred) RViz overlay nodes in ``drone_rescue_viz/``: each
      currently subscribes to its own ROS topic set with viz-specific
      fields (aruco_id, priority, detection_type, drones_surveying,
      DroneStatus.state) that aren't on MissionViewModel. Migration
      blocked on either extending MissionViewModel with those fields
      (architectural creep, since dashboard never reads them) or trimming
      them from the viz output.
    """

    def render_from(self, view: 'MissionViewModel') -> None:
        """Render the operator's view of ``view`` into the
        implementation's native medium.

        Implementations may assume ``view`` is a frozen ``MissionViewModel``
        instance, the immutable snapshot of mission state at this
        tick. Implementations must not mutate it.
        """
        ...


class SceneRenderer(OperatorView, Protocol):
    """Extended OperatorView for the dashboard's mission-scene tabs.

    Carries the trail-update path separately from the view-model path
    because trails are high-frequency append-only position history that
    doesn't fit the immutable-replace shape of ``MissionViewModel``.

    Concrete implementations: ``MissionSceneView`` (2D top-down plan,
    canonical SAR operator display) and ``Scene3DView``
    (pyqtgraph.opengl sand-table). ``DashboardWindow`` holds scene
    references typed by this Protocol, so adding a renderer is
    injection, not a tab-builder edit.
    """

    def render_trails(
        self,
        trails: Mapping[str, 'Deque[Tuple[float, float]]'],
        appended: Mapping[str, int],
    ) -> None:
        """Paint the per-drone trail histories.

        ``trails`` is the bounded ``(x, y)`` deque per drone.
        ``appended`` is the monotonic per-drone append counter used
        for change detection: implementations must compare against
        it, never against ``len(deque)``, which plateaus at maxlen
        (the bounded-deque regression class).
        """
        ...
