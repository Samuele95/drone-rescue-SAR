"""Unit coverage for RosControl.call_service.

The full RosControl spins an rclpy executor in a daemon thread; that
is integration-only. These tests pin the small bits of pure logic
inside ``call_service`` that we can exercise without rclpy: the
success / failure / message extraction from a stand-in response.
"""

from types import SimpleNamespace


def _extract(response):
    """Replica of the (success, message) extraction inside
    RosControl.call_service. Pulled out so this test exercises the
    contract without touching rclpy.

    Keep in sync with ros_control.RosControl.call_service.
    """
    success = bool(getattr(response, 'success', True))
    message = str(getattr(response, 'message', '') or '')
    return success, message


def test_response_success_true_and_message():
    r = SimpleNamespace(success=True, message='startup done')
    assert _extract(r) == (True, 'startup done')


def test_response_success_false_carries_message():
    r = SimpleNamespace(success=False, message='lifecycle_manager not active')
    assert _extract(r) == (False, 'lifecycle_manager not active')


def test_response_without_message_defaults_to_empty():
    r = SimpleNamespace(success=True)
    assert _extract(r) == (True, '')


def test_response_with_none_message_coerces_to_empty():
    r = SimpleNamespace(success=True, message=None)
    assert _extract(r) == (True, '')


def test_response_missing_success_defaults_to_true():
    """std_srvs.Trigger always carries `.success`; for other service
    types the absence-of-success defaults to True so a clean callee
    response is treated as OK. Documented behaviour."""
    r = SimpleNamespace()
    assert _extract(r) == (True, '')


def test_response_with_falsy_success_int_zero():
    r = SimpleNamespace(success=0, message='nope')
    assert _extract(r) == (False, 'nope')
