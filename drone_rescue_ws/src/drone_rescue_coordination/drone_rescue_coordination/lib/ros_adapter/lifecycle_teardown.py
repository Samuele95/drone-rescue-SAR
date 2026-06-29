"""Shared lifecycle teardown helper.

`drone_controller._safe_destroy(attr_name, destroy_fn)` collapses 13
near-identical try/except destroy_* blocks in
`drone_controller.on_cleanup`; `surveyor.on_cleanup` has the same
pattern (11 blocks). Lifting the helper to this shared module lets
every LifecycleNode in the coordination package consume it; future
promotions (battery_monitor, etc.) inherit the idiom for free.

The helper is namespaced as a free function, not a method on a
mixin, so consumers don't acquire a base-class dependency.
"""

from __future__ import annotations


def safe_destroy(node, attr_name: str, destroy_fn) -> None:
    """Read `node.<attr_name>`; if non-None, call `destroy_fn(value)`
    and null the attribute. Logs at WARN on any exception (matches the
    legacy per-block behaviour).

    `node` is any rclpy Node-like with `get_logger()` and the named
    attribute. `destroy_fn` is one of `node.destroy_timer`,
    `node.destroy_subscription`, `node.destroy_publisher`, etc.
    """
    try:
        value = getattr(node, attr_name, None)
        if value is not None:
            destroy_fn(value)
            setattr(node, attr_name, None)
    except Exception as e:
        node.get_logger().warning(
            f'Error destroying {attr_name}: {e}',
        )
