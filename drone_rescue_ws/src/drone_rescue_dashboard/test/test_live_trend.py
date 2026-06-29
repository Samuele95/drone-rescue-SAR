"""Live trend panel: TrendBuffer sampling/reset logic.

Pure pytest exercising the buffer behind LiveTrendWidget without a
running dashboard or QApplication.
"""

from __future__ import annotations

from drone_rescue_dashboard.widgets.live_trend import TrendBuffer


def test_records_advancing_samples():
    b = TrendBuffer()
    b.record(0.0, 0.0, 0)
    b.record(1.0, 12.0, 0)
    b.record(2.0, 25.0, 1)
    assert len(b) == 3
    # times/coverage/confirmed are bounded deques.
    assert list(b.times) == [0.0, 1.0, 2.0]
    assert list(b.coverage) == [0.0, 12.0, 25.0]
    assert list(b.confirmed) == [0, 0, 1]


def test_ignores_non_advancing_time():
    """A paused sim / repeated coverage message must not pile points."""
    b = TrendBuffer()
    b.record(5.0, 30.0, 1)
    b.record(5.0, 30.0, 1)   # same elapsed, ignored
    b.record(5.0, 31.0, 2)   # still same elapsed, ignored
    assert len(b) == 1


def test_time_running_backwards_resets_for_new_mission():
    b = TrendBuffer()
    b.record(10.0, 80.0, 4)
    b.record(20.0, 95.0, 5)
    assert len(b) == 2
    # New mission: sim clock restarts at a lower elapsed time.
    b.record(1.0, 3.0, 0)
    assert len(b) == 1
    assert list(b.times) == [1.0]
    assert list(b.coverage) == [3.0]


def test_buffer_is_bounded():
    b = TrendBuffer(maxlen=50)
    for i in range(200):
        b.record(float(i), float(i) / 2.0, i // 10)
    assert len(b) == 50
    # Oldest samples evicted; the newest survive in order.
    assert b.times[-1] == 199.0
    assert b.times[0] == 150.0
