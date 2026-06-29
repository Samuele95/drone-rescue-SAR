"""Driver and driven ports: the hexagonal boundary.

Each Protocol module carries a ``LAYER_BOUNDARY`` annotation aligning
the hexagonal boundary with the slides' 3T architecture (Marcelletti,
p. 33). Below the imports each module is grouped by its 3T layer; the
``__all__`` list mirrors the grouping for readers.

3T layer-boundary classification:

  Layer 3 (Deliberative, planning):
    DeliberativePlanner  : L3 task-planning interface
    WorldModel           : see lib/domain/world_model.py

  Layer 2 (Executive, sequencing, monitoring, exception-handling):
    MissionPort          : L2 ROS adapter to L3 Mission aggregate (in)
    ExecutiveSupervisor  : L2 to L3 escalation
    BehaviouralLayer     : L2 to L1 task dispatch
    ChangeStateClient    : L2 driving, ROS lifecycle transitions
    RecoveryDispatcher   : L2 output, recovery side-effects

  Layer 1 (Behavioural, close to sensors/actuators):
    SurveyorPort         : L1 motor-schema reducer driven port
    StigmergyPort        : L1 stigmergic substrate
    BehaviouralContext   : see lib/domain/behaviour_actions.py
                           (frozen L1 sensor context)

  Cross-cutting (infrastructure consumed across layers):
    Clock                : wall clock
    RngSource            : seeded RNG
    EventPort            : operator-event sink
    Bidder/BidderRegistry : auction structural interface

Hexagonal invariant: ports declare contracts; production adapters
live in ``lib/ros_adapter/``. No rclpy in this package. Tests can
implement Protocols with SimpleNamespace.
"""

# Layer 3: Deliberative
from .deliberative_planner import DeliberativePlanner
# Layer 3: Organisation (Unit-10 distributed-goal layer)
from .motivation import Desire, Motivation, MotivationContext

# Layer 2: Executive
from .affect_monitor import (
    AffectMonitor,
    ExploitationSample,
    StuckSignal,
)
from .behavioural_layer import BehaviouralLayer
from .change_state_client import ChangeStateClient
from .executive_supervisor import (
    ExecutiveSupervisor,
    InMemoryExecutiveCapture,
)
from .mission_port import MissionPort
from .recovery_dispatcher import RecoveryDispatcher

# Layer 1: Behavioural
from .arbitration import ArbitrationStrategy
from .behaviour import Behaviour
from .stigmergy_port import InMemoryStigmergyGrid, StigmergyPort
from .surveyor_port import (
    SurveyorOutputs,
    SurveyorPort,
    SurveyorSensors,
)

# Cross-cutting infrastructure
from .bidder_registry import Bidder, BidderRegistry, DictBidderRegistry
from .clock import Clock, FakeClock
# Decentralisation skeleton; live gossip adapter deferred.
from .peer_state import (
    PeerGossipUpdate,
    PeerSnapshot,
    PeerStateRegistry,
)
# Protocol + in-memory test fake only; the production
# ``RosEventPublisherAdapter`` lives in
# ``lib.ros_adapter.event_publisher`` (ports declare contracts,
# adapters live in ros_adapter/).
from .event_port import EventPort, InMemoryEventCapture
from .rng_source import RngSource

__all__ = [
    # L3
    'DeliberativePlanner',
    # L3 organisation (Unit-10)
    'Motivation', 'Desire', 'MotivationContext',
    # L2
    'MissionPort',
    'ExecutiveSupervisor', 'InMemoryExecutiveCapture',
    'BehaviouralLayer',
    'AffectMonitor', 'ExploitationSample', 'StuckSignal',
    'ChangeStateClient',
    'RecoveryDispatcher',
    # L1
    'SurveyorPort', 'SurveyorSensors', 'SurveyorOutputs',
    'StigmergyPort', 'InMemoryStigmergyGrid',
    'Behaviour', 'ArbitrationStrategy',
    # Cross-cutting
    'EventPort', 'InMemoryEventCapture',
    'Clock', 'FakeClock',
    'RngSource',
    'Bidder', 'BidderRegistry', 'DictBidderRegistry',
    'PeerStateRegistry', 'PeerSnapshot', 'PeerGossipUpdate',
]
