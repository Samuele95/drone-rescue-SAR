"""In-process ROS control for Mission Control.

The legacy path shells out to
``subprocess.run(['ros2', 'param', 'set', node, param, value])`` once
per runtime parameter, plus another ``subprocess.run`` for
``ros2 topic pub --once /survey/start``. With ~14 runtime params
(mission_manager + detection_filter) that's 14 x ~100-300 ms of CLI
startup overhead per Run = 1.4-4 s of avoidable latency between the
"All nodes activated" sentinel and the actual ``/survey/start``
publish.

This module spins a daemon-thread rclpy executor inside the Qt
process and exposes a ``RosControl`` driver-port adapter that batches
param sets through ``rclpy.AsyncParameterClient`` plus a one-shot
publisher. All work collapses to tens of ms.

Designed for clean shutdown: `RosControl.shutdown()` joins the
spin thread and destroys the node. Mission Control's
``MissionControlWindow.closeEvent`` calls it.
"""

from __future__ import annotations

import threading
from typing import Iterable, List, Optional, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool


_NODE_NAME = 'mission_control_in_process'

# /survey/start QoS: match readiness_coordinator + mission_manager
# subscriber so the latched value reaches the right places.
_SURVEY_START_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)


class RosControl:
    """Driver-port adapter Mission Control uses to set ROS params and
    publish operator triggers without shelling out to the ros2 CLI.

    The adapter owns a long-lived rclpy node spinning in a daemon
    thread. Construction is lazy from Mission Control's main thread;
    the first ``set_params`` or ``publish_survey_start`` call ensures
    rclpy is initialised + the node + executor + spin thread are up.
    Subsequent calls are sub-millisecond.

    Shutdown: ``shutdown()`` is idempotent; call it from
    ``QMainWindow.closeEvent`` to join the spin thread cleanly.
    """

    def __init__(self) -> None:
        self._node: Optional[Node] = None
        self._executor: Optional[MultiThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._survey_start_pub = None
        self._owns_rclpy: bool = False
        self._lock = threading.Lock()

    # ----------------------------------------------------------- lazy init
    def _ensure_started(self) -> None:
        with self._lock:
            if self._node is not None:
                return
            if not rclpy.ok():
                rclpy.init()
                self._owns_rclpy = True
            self._node = Node(_NODE_NAME)
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self._node)
            self._spin_thread = threading.Thread(
                target=self._executor.spin,
                daemon=True,
                name='mission_control_rclpy_spin',
            )
            self._spin_thread.start()
            # Pre-create the /survey/start publisher so the first publish
            # has zero discovery latency.
            self._survey_start_pub = self._node.create_publisher(
                Bool, '/survey/start', _SURVEY_START_QOS,
            )

    # ----------------------------------------------------------- params
    def set_params(
        self,
        target_node: str,
        params: Iterable[Tuple[str, object]],
        timeout_s: float = 8.0,
    ) -> List[Tuple[str, bool, str]]:
        """Set a batch of parameters on a remote ROS node.

        Returns a list of ``(param_name, success, message)`` tuples,
        the same shape per-param the legacy subprocess path produced.

        Uses the synchronous ``rclpy.parameter_service`` client (which
        ``AsyncParameterClient`` wraps) so the call returns when the
        remote node has acked. With the in-process spinner already
        running, total latency for ~14 params is tens of ms instead
        of seconds.
        """
        self._ensure_started()
        from rclpy.parameter_client import AsyncParameterClient

        client = AsyncParameterClient(self._node, target_node)
        if not client.wait_for_services(timeout_sec=timeout_s):
            return [(name, False, f'service /{target_node}/set_parameters '
                     f'unavailable within {timeout_s}s')
                    for name, _ in params]

        results: List[Tuple[str, bool, str]] = []
        for name, value in params:
            try:
                fut = client.set_parameters([Parameter(name, value=value)])
                # Wait synchronously for the future to complete (the
                # spin thread drives the executor; we just block).
                end = self._node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
                while not fut.done():
                    if self._node.get_clock().now().nanoseconds >= end:
                        results.append((name, False, 'timeout'))
                        break
                    threading.Event().wait(0.005)
                else:
                    response = fut.result()
                    if response and response.results and response.results[0].successful:
                        results.append((name, True, ''))
                    else:
                        reason = (response.results[0].reason
                                  if response and response.results else 'unknown')
                        results.append((name, False, reason))
            except Exception as e:
                results.append((name, False, f'{type(e).__name__}: {e}'))
        return results

    # ----------------------------------------------------------- generic service call
    def call_service(
        self,
        service_name: str,
        service_type,
        request=None,
        timeout_s: float = 30.0,
    ) -> Tuple[bool, str]:
        """Synchronous in-process service call.

        Closes the last ``subprocess.run`` site in the
        Mission Control / Qt process (the
        ``_fallback_lifecycle_startup`` shell-out to ``ros2 service
        call``). Uses the already-running in-process node + spin
        thread; one extra service client per call (created+destroyed
        because this is a rare, one-shot path).
        """
        self._ensure_started()
        if request is None:
            request = service_type.Request()
        client = self._node.create_client(service_type, service_name)
        try:
            if not client.wait_for_service(timeout_sec=timeout_s):
                return (False,
                        f'service {service_name} unavailable within {timeout_s}s')
            fut = client.call_async(request)
            end = self._node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
            while not fut.done():
                if self._node.get_clock().now().nanoseconds >= end:
                    return (False, 'timeout')
                threading.Event().wait(0.005)
            response = fut.result()
            if response is None:
                return (False, 'no response')
            # std_srvs/srv/Trigger response carries (success, message).
            success = bool(getattr(response, 'success', True))
            message = str(getattr(response, 'message', '') or '')
            return (success, message)
        except Exception as e:
            return (False, f'{type(e).__name__}: {e}')
        finally:
            try:
                self._node.destroy_client(client)
            except Exception:
                pass

    # ----------------------------------------------------------- triggers
    def publish_survey_start(self) -> None:
        """Publish ``/survey/start`` once, latched (TRANSIENT_LOCAL).

        Replaces the legacy ``ros2 topic pub --once`` shell-out.
        Latched durability means a late-joining mission_manager picks
        the trigger up on subscribe.
        """
        self._ensure_started()
        if self._survey_start_pub is None:
            return
        self._survey_start_pub.publish(Bool(data=True))

    # ----------------------------------------------------------- shutdown
    def shutdown(self) -> None:
        """Tear down the in-process node and join the spin thread.

        Idempotent. Mission Control's ``closeEvent`` should call this.
        """
        with self._lock:
            if self._node is None:
                return
            try:
                if self._executor is not None:
                    self._executor.shutdown()
            except Exception:
                pass
            try:
                self._node.destroy_node()
            except Exception:
                pass
            if self._spin_thread is not None:
                self._spin_thread.join(timeout=2.0)
            if self._owns_rclpy and rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception:
                    pass
            self._node = None
            self._executor = None
            self._spin_thread = None
            self._survey_start_pub = None
            self._owns_rclpy = False
