"""Audit test: every Protocol module in lib/ports/ carries a
LAYER_BOUNDARY 3T annotation.

The annotation is the machine-readable surface a future CI rule could
check (e.g. "domain modules must not import from L2-output ports").

Pure-Python; no rclpy.
"""
from __future__ import annotations

from drone_rescue_coordination.lib.ports import (
    affect_monitor, arbitration, behaviour, behavioural_layer,
    bidder_registry, change_state_client, clock, deliberative_planner,
    event_port, executive_supervisor, mission_port, motivation,
    peer_state, recovery_dispatcher, rng_source, sector_owner_policy,
    stigmergy_port, surveyor_port,
)


VALID_BOUNDARIES = {
    'L1', 'L1-driven', 'L1-stigmergy',   # 'L1' = L1-internal (basis behaviours, arbitration)
    'L1-L2',
    'L2-driving', 'L2-output',
    'L2-L3',
    'L3-internal',                        # planner-internal policy (sector ownership)
    'L3-organisation',
    'cross-cutting',
}


PORT_MODULES = {
    'affect_monitor':       (affect_monitor,       'L2-L3'),
    'arbitration':          (arbitration,          'L1'),
    'behaviour':            (behaviour,            'L1'),
    'behavioural_layer':    (behavioural_layer,    'L1-L2'),
    'bidder_registry':      (bidder_registry,      'cross-cutting'),
    'change_state_client':  (change_state_client,  'L2-driving'),
    'clock':                (clock,                'cross-cutting'),
    'deliberative_planner': (deliberative_planner, 'L2-L3'),
    'event_port':           (event_port,           'cross-cutting'),
    'executive_supervisor': (executive_supervisor, 'L2-L3'),
    'mission_port':         (mission_port,         'L2-L3'),
    'motivation':           (motivation,           'L3-organisation'),
    'peer_state':           (peer_state,           'cross-cutting'),
    'recovery_dispatcher':  (recovery_dispatcher,  'L2-output'),
    'rng_source':           (rng_source,           'cross-cutting'),
    'sector_owner_policy':  (sector_owner_policy,  'L3-internal'),
    'stigmergy_port':       (stigmergy_port,       'L1-stigmergy'),
    'surveyor_port':        (surveyor_port,        'L1-driven'),
}


def test_every_port_module_has_layer_boundary():
    """No port module is allowed to ship without a 3T annotation;
    this is the CI surface for the boundary check."""
    for name, (module, _) in PORT_MODULES.items():
        assert hasattr(module, 'LAYER_BOUNDARY'), (
            f'lib/ports/{name}.py is missing LAYER_BOUNDARY (F10)'
        )


def test_every_layer_boundary_is_a_known_label():
    for name, (module, _) in PORT_MODULES.items():
        boundary = module.LAYER_BOUNDARY
        assert boundary in VALID_BOUNDARIES, (
            f'lib/ports/{name}.py has unknown LAYER_BOUNDARY {boundary!r}; '
            f'expected one of {sorted(VALID_BOUNDARIES)}'
        )


def test_classifications_match_expected():
    """Each port's actual annotation matches the expected
    classification documented in lib/ports/__init__.py."""
    for name, (module, expected) in PORT_MODULES.items():
        assert module.LAYER_BOUNDARY == expected, (
            f'lib/ports/{name}.py is classified as {module.LAYER_BOUNDARY!r}; '
            f'expected {expected!r} per F10'
        )
