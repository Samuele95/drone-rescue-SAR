"""Save As scenario: save_scenario() round-trip.

Pure pytest; no Qt. The Setup tab's "Save As…" button persists the
operator's current form values via save_scenario(); this test pins that
a saved scenario reloads byte-faithfully through load_scenario().
"""

from __future__ import annotations

import pytest

from drone_rescue_mission_control.scenario_loader import (
    ScenarioVictim,
    load_scenario,
    save_scenario,
)


def test_save_round_trips_all_blocks(tmp_path):
    path = tmp_path / 'my_scenario.yaml'
    saved = save_scenario(
        path,
        name='My Scenario',
        description='hand-tuned variant',
        seed=7,
        launch={'num_drones': 3, 'coverage_pattern': 'parallel_track'},
        mission={'mission_radius': 55.0, 'survey_altitude': 30.0},
        detection={'confidence_floor': 0.7},
        ground_truth_victims=[
            ScenarioVictim(id=1, position=(10.0, -20.0, 0.0)),
            ScenarioVictim(id=2, position=(5.0, 5.0, 0.0)),
        ],
    )
    # save_scenario returns the reloaded VO.
    assert saved.name == 'My Scenario'
    assert saved.seed == 7
    assert saved.launch['num_drones'] == 3
    assert saved.mission['mission_radius'] == 55.0
    assert saved.detection['confidence_floor'] == 0.7
    assert len(saved.ground_truth_victims) == 2

    # And the file on disk reloads identically.
    reloaded = load_scenario(path)
    assert reloaded.name == 'My Scenario'
    assert reloaded.description == 'hand-tuned variant'
    assert reloaded.seed == 7
    assert reloaded.launch == {'num_drones': 3,
                               'coverage_pattern': 'parallel_track'}
    assert reloaded.mission == {'mission_radius': 55.0,
                                'survey_altitude': 30.0}
    assert reloaded.detection == {'confidence_floor': 0.7}
    assert reloaded.ground_truth_victims[0].id == 1
    assert reloaded.ground_truth_victims[0].position == (10.0, -20.0, 0.0)


def test_save_minimal_scenario(tmp_path):
    """A scenario with only a name still saves and reloads."""
    path = tmp_path / 'bare.yaml'
    save_scenario(path, name='Bare')
    reloaded = load_scenario(path)
    assert reloaded.name == 'Bare'
    assert reloaded.launch == {}
    assert reloaded.ground_truth_victims == []


def test_save_rejects_unknown_param_and_cleans_up(tmp_path):
    """An invalid key fails validation on the round-trip guard, and the
    partial file is removed rather than left unloadable on disk."""
    path = tmp_path / 'bad.yaml'
    with pytest.raises(Exception):
        save_scenario(path, name='Bad', mission={'not_a_real_param': 1})
    assert not path.exists()
