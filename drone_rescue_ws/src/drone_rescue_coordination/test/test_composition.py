"""Unit coverage for the process-wide CompositionRoot.

The class body is pure-Python: these tests exercise the wiring
contract (factory + back-compat default) without ``rclpy.init()``.
The 8-node integration is exercised by the existing per-node test
suite; this file just pins the composition seam.
"""

from drone_rescue_coordination.lib.composition import CompositionRoot


class _FakeClock:
    def now_sec(self) -> float:
        return 0.0


class _FakeTopicFactory:
    def __init__(self, *, label):
        self.label = label


# defaults

def test_composition_root_all_fields_default_none():
    c = CompositionRoot()
    assert c.clock is None
    assert c.event_port is None
    assert c.topic_factory is None
    assert c.scenario_repo is None
    assert c.parameter_declarer is None


def test_composition_root_accepts_partial_construction():
    """Tests construct a sparse composition with only the adapters
    they care about."""
    c = CompositionRoot(clock=_FakeClock())
    assert c.clock is not None
    assert c.event_port is None


def test_composition_root_carries_all_supplied_adapters():
    fake_clock = _FakeClock()
    fake_tf = _FakeTopicFactory(label='hot')
    c = CompositionRoot(
        clock=fake_clock,
        topic_factory=fake_tf,
        event_port='ep',
        scenario_repo='sr',
        parameter_declarer='pd',
    )
    assert c.clock is fake_clock
    assert c.topic_factory is fake_tf
    assert c.event_port == 'ep'
    assert c.scenario_repo == 'sr'
    assert c.parameter_declarer == 'pd'


def test_composition_root_is_dataclass():
    """Field assignment must be settable post-construction (the back-
    compat path lets tests build incrementally)."""
    c = CompositionRoot()
    c.clock = _FakeClock()
    assert c.clock is not None


# bind_composition

def test_bind_composition_picks_up_drone_names_from_node():
    """bind_composition reads drone_names from the node attribute when
    not passed explicitly, then writes the new CompositionRoot back to
    node._composition."""
    from drone_rescue_coordination.lib.composition import bind_composition

    class _FakeNode:
        def __init__(self):
            self.drone_names = ['drone_a', 'drone_b']
            self._composition = None

        # CompositionRoot.for_node calls these on the node; provide
        # stubs so the helper exercises end-to-end without rclpy.
        def create_publisher(self, *args, **kwargs):
            class _Pub:
                def publish(self, *_a, **_k):
                    pass
            return _Pub()

        def get_clock(self):
            class _C:
                def now(self):
                    class _N:
                        nanoseconds = 0
                    return _N()
            return _C()

    n = _FakeNode()
    returned = bind_composition(
        n,
        with_event_port=False,         # avoid rclpy QoS dependency
        with_scenario_repo=False,
        with_parameter_declarer=False,
    )
    assert returned is n
    assert n._composition is not None
    assert n._composition.clock is not None
    assert n._composition.topic_factory is not None


def test_bind_composition_skips_topic_factory_without_drone_names():
    from drone_rescue_coordination.lib.composition import bind_composition

    class _FakeNode:
        _composition = None

        def get_clock(self):
            class _C:
                def now(self):
                    class _N:
                        nanoseconds = 0
                    return _N()
            return _C()

    n = _FakeNode()
    bind_composition(
        n,
        with_event_port=False,
        with_scenario_repo=False,
        with_parameter_declarer=False,
    )
    # Topic factory needs drone_names; without it the field is None.
    assert n._composition.topic_factory is None


def _fake_node():
    class _FakeNode:
        _composition = None

        def get_clock(self):
            class _C:
                def now(self):
                    class _N:
                        nanoseconds = 0
                    return _N()
            return _C()
    return _FakeNode()


def test_scenario_repo_is_injected_not_constructed():
    """The composition no longer constructs the scenario repository by
    importing mission_control; the adapter is injected and threaded through."""
    from drone_rescue_coordination.lib.composition import bind_composition
    sentinel = object()
    n = _fake_node()
    bind_composition(
        n, with_event_port=False, with_parameter_declarer=False,
        scenario_repo=sentinel,
    )
    assert n._composition.scenario_repo is sentinel


def test_scenario_repo_default_is_none():
    """The default path constructs no scenario repo (no inversion); the
    mission_control layer injects or builds its own."""
    from drone_rescue_coordination.lib.composition import bind_composition
    n = _fake_node()
    bind_composition(n, with_event_port=False, with_parameter_declarer=False)
    assert n._composition.scenario_repo is None


def test_composition_module_does_not_import_mission_control():
    """Static boundary check: coordination's composition has no import edge into
    drone_rescue_mission_control (the dependency must point the other way)."""
    import ast
    import inspect
    from drone_rescue_coordination.lib import composition as comp

    tree = ast.parse(inspect.getsource(comp))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                assert not a.name.startswith('drone_rescue_mission_control')
        elif isinstance(node, ast.ImportFrom):
            assert not (node.module or '').startswith('drone_rescue_mission_control')
