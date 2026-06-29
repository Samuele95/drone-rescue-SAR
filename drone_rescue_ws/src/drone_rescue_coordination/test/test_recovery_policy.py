"""Unit coverage for the lifted RecoveryPolicy.

The policy is pure-Python: these tests exercise the dispatch logic
through an ``InMemoryRecoveryRecorder`` without any rclpy or
LifecycleNode dependency. Each NodeKind branch has explicit coverage.
"""

from drone_rescue_coordination.lib.domain.system_mode_machine import SystemMode
from drone_rescue_coordination.lib.lifecycle.recovery_policy import (
    RecoveryPolicy,
)
from drone_rescue_coordination.lib.ports.recovery_dispatcher import (
    InMemoryRecoveryRecorder,
)


def test_handle_pheromone_triggers_safe_mode():
    rec = InMemoryRecoveryRecorder()
    p = RecoveryPolicy(
        dispatcher=rec,
        unresponsive_provider=lambda: (),
        mode_provider=lambda: SystemMode.NORMAL,
    )
    p.handle_pheromone('pheromone_server')
    assert rec.calls == [
        ('trigger_safe_mode', 'Pheromone server pheromone_server unresponsive'),
    ]


def test_handle_controller_lands_one_drone_no_escalation():
    rec = InMemoryRecoveryRecorder()
    p = RecoveryPolicy(
        dispatcher=rec,
        unresponsive_provider=lambda: ('drone1_controller',),  # only one
        mode_provider=lambda: SystemMode.NORMAL,
    )
    p.handle_controller('drone1_controller')
    assert rec.calls == [('command_drone_land', 'drone1')]


def test_handle_controller_escalates_to_degraded_on_two_controllers():
    rec = InMemoryRecoveryRecorder()
    p = RecoveryPolicy(
        dispatcher=rec,
        unresponsive_provider=lambda: (
            'drone1_controller', 'drone2_controller',
        ),
        mode_provider=lambda: SystemMode.NORMAL,
    )
    p.handle_controller('drone2_controller')
    assert rec.calls == [
        ('command_drone_land', 'drone2'),
        ('transition_to', SystemMode.DEGRADED,
         '2 drone controllers unresponsive'),
    ]


def test_handle_controller_with_hyphen_node_name():
    """Some launch files spawn controllers as `drone1-controller` (hyphen
    separator) instead of `drone1_controller`; both paths must work."""
    rec = InMemoryRecoveryRecorder()
    p = RecoveryPolicy(
        dispatcher=rec,
        unresponsive_provider=lambda: ('drone1-controller',),
        mode_provider=lambda: SystemMode.NORMAL,
    )
    p.handle_controller('drone1-controller')
    assert rec.calls == [('command_drone_land', 'drone1')]


def test_handle_controller_ignores_unrelated_unresponsive_nodes():
    """The escalation count uses 'controller' substring match: a
    stray unresponsive surveyor / executor must not push us over the
    threshold."""
    rec = InMemoryRecoveryRecorder()
    p = RecoveryPolicy(
        dispatcher=rec,
        unresponsive_provider=lambda: (
            'drone1_controller', 'surveyor_node', 'executor_d2',
        ),
        mode_provider=lambda: SystemMode.NORMAL,
    )
    p.handle_controller('drone1_controller')
    # Only one *controller* is unresponsive; no escalation.
    assert rec.calls == [('command_drone_land', 'drone1')]


def test_handle_controller_known_fleet_unknown_id_logs_and_skips():
    """If ROS namespace remapping makes the hardware_id arrive as
    'fleet/drone1-controller', the extracted drone_id is 'fleet/drone1'
    which won't match any known drone; the policy should log and skip
    rather than silently calling command_drone_land() on the bogus id."""
    rec = InMemoryRecoveryRecorder()
    warnings = []
    p = RecoveryPolicy(
        dispatcher=rec,
        unresponsive_provider=lambda: ('fleet/drone1-controller',),
        mode_provider=lambda: SystemMode.NORMAL,
        known_drone_names=('drone1', 'drone2', 'drone3', 'drone4'),
        logger_fn=warnings.append,
    )
    p.handle_controller('fleet/drone1-controller')
    assert rec.calls == []   # no command_drone_land called
    assert len(warnings) == 1
    assert 'fleet/drone1' in warnings[0]


def test_handle_controller_known_fleet_valid_id_proceeds():
    """When the drone_id matches the known fleet, the land command
    proceeds normally."""
    rec = InMemoryRecoveryRecorder()
    p = RecoveryPolicy(
        dispatcher=rec,
        unresponsive_provider=lambda: ('drone1_controller',),
        mode_provider=lambda: SystemMode.NORMAL,
        known_drone_names=('drone1', 'drone2', 'drone3', 'drone4'),
    )
    p.handle_controller('drone1_controller')
    assert rec.calls == [('command_drone_land', 'drone1')]


def test_handle_executor_or_surveyor_escalates_from_normal():
    rec = InMemoryRecoveryRecorder()
    p = RecoveryPolicy(
        dispatcher=rec,
        unresponsive_provider=lambda: (),
        mode_provider=lambda: SystemMode.NORMAL,
    )
    p.handle_executor_or_surveyor('executor_drone1')
    assert rec.calls == [
        ('transition_to', SystemMode.DEGRADED,
         'Executor/surveyor executor_drone1 unresponsive'),
    ]


def test_handle_executor_or_surveyor_no_op_when_already_safe():
    """Already-SAFE mode must not re-trigger a DEGRADED transition
    (the policy gates on ``current_mode == NORMAL``)."""
    rec = InMemoryRecoveryRecorder()
    p = RecoveryPolicy(
        dispatcher=rec,
        unresponsive_provider=lambda: (),
        mode_provider=lambda: SystemMode.SAFE,
    )
    p.handle_executor_or_surveyor('executor_drone1')
    assert rec.calls == []


def test_handle_executor_or_surveyor_no_op_when_already_degraded():
    rec = InMemoryRecoveryRecorder()
    p = RecoveryPolicy(
        dispatcher=rec,
        unresponsive_provider=lambda: (),
        mode_provider=lambda: SystemMode.DEGRADED,
    )
    p.handle_executor_or_surveyor('surveyor_node')
    assert rec.calls == []


def test_handle_unknown_is_silent():
    rec = InMemoryRecoveryRecorder()
    p = RecoveryPolicy(
        dispatcher=rec,
        unresponsive_provider=lambda: (),
        mode_provider=lambda: SystemMode.NORMAL,
    )
    p.handle_unknown('mystery_node')
    assert rec.calls == []
