"""CompositionRoot: adapter-wiring seam for the coordination process.

Per-LifecycleNode helper that bundles the hexagonal
adapter constructions in one place. Each LifecycleNode constructor
now accepts ``composition: Optional[CompositionRoot] = None``; when
present the node reads its adapters off the composition root, when
None the node falls back to inline lazy construction (back-compat
for existing tests).

Adding a 9th adapter is a one-line edit in
``CompositionRoot.for_node``: the 8 LifecycleNodes never need to
import the new adapter directly.

Pure-Python: the rclpy-dependent adapter constructions live behind
the ``for_node(node)`` factory so the class body itself is rclpy-free.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Callable, List, Optional

if TYPE_CHECKING:
    # Typed Port-Protocol field annotations.
    # ``from __future__ import annotations`` + TYPE_CHECKING means
    # these imports never execute at runtime, so the class body stays
    # rclpy-free while static analysis (mypy / pyright) gets the
    # structural-type information it needs to catch mis-wiring.
    from .ports.affect_monitor import AffectMonitor
    from .ports.behavioural_layer import BehaviouralLayer
    from .ports.clock import Clock
    from .ports.event_port import EventPort
    from .ports.stigmergy_port import StigmergyPort
    from .ros_adapter.topic_factory import TopicFactory


@dataclass
class CompositionRoot:
    """Adapter-wiring bundle. All fields optional so tests can construct
    a sparse composition with only the adapters under test.

    Field types are typed against the existing Port Protocols; these are
    structural-typing Protocols with no rclpy imports
    so the class body remains rclpy-free even after the upgrade.
    ``scenario_repo`` and ``parameter_declarer`` are typed broadly
    (``Any`` / callable) because the scenario-repo Protocol lives in
    a different package and the parameter declarer is a module-level
    function rather than an instance.
    """
    clock: Optional['Clock'] = None
    event_port: Optional['EventPort'] = None
    topic_factory: Optional['TopicFactory'] = None
    scenario_repo: Optional[Any] = None
    parameter_declarer: Optional[Callable[..., None]] = None
    # Typed StigmergyPort field for the
    # Surveyor's pheromone medium. Per-drone instance:
    # for_node() requires a drone_name to wire the
    # RosPheromoneAdapter. Tests can pass an InMemoryStigmergyGrid.
    stigmergy_port: Optional['StigmergyPort'] = None
    # Typed BehaviouralLayer
    # injection point. In the deployed system the L2->L1 boundary is
    # the ROS ``/<drone>/task`` topic (DroneExecutor subscribes), so
    # production leaves this None and ``for_node`` constructs no
    # adapter for it. The field exists so an in-process integration
    # test or research harness can inject an alternative L1: a
    # FakeBehaviouralLayer that records dispatch_task calls, a
    # subsumption shell, or a learned-policy rollout, without
    # spinning up a ROS node. Completes the composition root's
    # coverage of the L1 ports.
    behavioural_layer: Optional['BehaviouralLayer'] = None
    # Typed AffectMonitor injection
    # point. The "emotion-as-model-free-stuck-detector":
    # consumes ``ExploitationSample`` from the executive tick and
    # exposes ``frustration(key)`` / ``is_stuck(key)``. Production
    # wiring (instantiating ExploitationTracker, calling observe in
    # the mission_manager / drone_executor tick, escalating a
    # ``StuckSignal`` through ``ExecutiveSupervisor``) is gated on a
    # live Gazebo run; in tests and research harnesses the field
    # injects an ``ExploitationTracker`` directly.
    affect_monitor: Optional['AffectMonitor'] = None

    @classmethod
    def for_node(cls, node, *,
                 drone_names: Optional[List[str]] = None,
                 with_event_port: bool = True,
                 with_scenario_repo: bool = False,
                 scenario_repo: Optional[Any] = None,
                 with_parameter_declarer: bool = True,
                 with_stigmergy_port: bool = False,
                 stigmergy_drone_name: Optional[str] = None,
                 ) -> 'CompositionRoot':
        """Factory: wires the production adapter set for one
        LifecycleNode.

        Defaults: every adapter is constructed. Callers that don't
        need a particular adapter pass ``with_X=False`` to skip its
        construction (saves the ROS publisher / file IO cost).

        ``drone_names`` is required by TopicFactory; pass None to
        skip topic-factory construction.

        Adding a 9th adapter is a one-line factory edit below.
        """
        from .ros_adapter.ros_clock import RosClock
        clock = RosClock(node)

        topic_factory = None
        if drone_names is not None:
            from .ros_adapter.topic_factory import TopicFactory
            topic_factory = TopicFactory(node, drone_names)

        # event_port via shared MissionEvent
        # publisher (matches the QoS the LifecycleNodes use inline:
        # RELIABLE, VOLATILE, depth=50).
        event_port = None
        if with_event_port:
            from rclpy.qos import (
                DurabilityPolicy, QoSProfile, ReliabilityPolicy,
            )
            from drone_rescue_msgs.msg import MissionEvent
            from .ros_adapter.event_publisher import RosEventPublisherAdapter
            event_pub = node.create_publisher(
                MissionEvent, '/mission/events',
                QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    depth=50,
                ),
            )
            event_port = RosEventPublisherAdapter(event_pub)

        # The scenario repository is INJECTED by the mission_control
        # layer (mission_recorder), not constructed here: coordination must not
        # import drone_rescue_mission_control (that is the wrong dependency
        # direction; mission_control depends on coordination). The contract is
        # the ScenarioRepository port (lib/ports/scenario_repository). Pass the
        # adapter straight through; ``with_scenario_repo`` is retained only as a
        # deprecated no-op for back-compat (default now False): coordination
        # never builds the adapter, so when no repo is injected mission_recorder
        # falls back to constructing its own YamlScenarioRepository.
        # (``with_scenario_repo`` is accepted but ignored; ``scenario_repo`` is
        # the injected adapter, threaded straight into the CompositionRoot.)
        _ = with_scenario_repo

        # ParameterDeclarer is the module-level
        # `declare_for_scope` callable; the composition exposes a
        # reference to it so nodes can call
        # `self._composition.parameter_declarer(self, ParamScope.X)`
        # uniformly.
        parameter_declarer = None
        if with_parameter_declarer:
            from .ros_adapter.parameter_declarer import declare_for_scope
            parameter_declarer = declare_for_scope

        # Opt-in per-drone stigmergy
        # adapter. The Surveyor LifecycleNode is the only consumer
        # today; opt-in (default False) so other nodes don't pay for a
        # subscription they won't read.
        stigmergy_port = None
        if with_stigmergy_port and stigmergy_drone_name is not None:
            from .ros_adapter.pheromone_adapter import RosPheromoneAdapter
            stigmergy_port = RosPheromoneAdapter.bound(
                node, stigmergy_drone_name,
            )

        return cls(
            clock=clock,
            topic_factory=topic_factory,
            event_port=event_port,
            scenario_repo=scenario_repo,
            parameter_declarer=parameter_declarer,
            stigmergy_port=stigmergy_port,
        )


def resolve_clock(node, composition):
    """Return the Clock the node should use.

    Collapses the duplicated ternary across 8
    LifecycleNodes::

        self._time = (
            self._composition.clock
            if self._composition is not None
            and self._composition.clock is not None
            else RosClock(self)
        )

    into one call::

        self._time = resolve_clock(self, self._composition)

    Tests that pass ``composition=CompositionRoot(clock=FakeClock())``
    still get their fake; tests that pass nothing get a RosClock bound
    to the (test) node.
    """
    if composition is not None and composition.clock is not None:
        return composition.clock
    from .ros_adapter.ros_clock import RosClock
    return RosClock(node)


def bind_composition(node, **for_node_kwargs):
    """Wire a CompositionRoot onto a freshly-constructed LifecycleNode.

    The ergonomic wrapper that makes binding a one-line edit in each
    ``main()``. Reads ``node.drone_names``
    when present (the LifecycleNode declared the parameter) and
    constructs the production composition via
    ``CompositionRoot.for_node(node, drone_names=...)``. Returns the
    node so call sites read as ``node = bind_composition(NodeCls())``.

    For the Surveyor LifecycleNode, the
    per-drone ``drone_name`` parameter selects which pheromone deposit
    topic to publish on. ``bind_composition`` detects the surveyor
    by node class name (``Surveyor``) and opts in to the
    StigmergyPort adapter. Other nodes don't pay for the
    subscription.

    Tests that need a sparse composition still construct
    ``CompositionRoot(...)`` directly and pass it via the
    ``composition=`` constructor kwarg.
    """
    drone_names = for_node_kwargs.pop('drone_names', None)
    if drone_names is None:
        drone_names = getattr(node, 'drone_names', None)

    # Opt-in stigmergy adapter for the Surveyor node, where the
    # pheromone medium is L1 architectural infrastructure.
    if (type(node).__name__ == 'Surveyor'
            and 'with_stigmergy_port' not in for_node_kwargs):
        for_node_kwargs['with_stigmergy_port'] = True
        if 'stigmergy_drone_name' not in for_node_kwargs:
            for_node_kwargs['stigmergy_drone_name'] = getattr(
                node, 'drone_name', None,
            )

    node._composition = CompositionRoot.for_node(
        node, drone_names=drone_names, **for_node_kwargs,
    )
    return node
