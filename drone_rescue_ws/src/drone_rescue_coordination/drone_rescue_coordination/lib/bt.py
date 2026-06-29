"""Tiny hand-rolled Behavior Tree primitives: Sequence, Selector, Action,
Condition, sufficient for drone_executor's per-drone decision logic.

This avoids pulling in py_trees / py_trees_ros (heavy deps) and lets us keep
the executor's tree fully under unit-test coverage.

Ticks return ``(Status, output)`` where Status is SUCCESS / FAILURE /
RUNNING and ``output`` is an opaque per-tick payload bubbled up from the
leaf that determined the result (None for control nodes / conditions).
RUNNING short-circuits parents that care about it (Sequence with running
children pauses; Selector with a running child does not advance).

The output channel lets leaf actions return a value
(the executor passes a ``BehaviouralOutput``) instead of mutating shared
context, so the BT stays a pure ``ctx -> (Status, output)`` evaluation.
The payload type is ``Any`` here: the generic BT does not depend on the
domain output type; only the composing layer interprets it.

References:
  * Colledanchise & Ögren, "Behavior Trees in Robotics and AI: An Introduction"
    CRC Press 2018.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from enum import IntEnum
from typing import Any, Callable, Hashable, List, Optional, Protocol, Tuple


class Status(IntEnum):
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3


# Every tick yields a status plus an opaque output
# payload (the leaf's behavioural output, or None for control nodes).
TickResult = Tuple[Status, Optional[Any]]


# Typed leaf callable contracts. The Action / Condition /
# Switch constructors below accept these Protocols so a swapped signature
# fails type-check rather than at first BT tick. Existing free-function
# call sites satisfy the Protocols structurally, no caller change
# required.

class ActionFn(Protocol):
    """`(ctx) -> (Status, output)`: the contract Action expects.
    The action returns its behavioural output alongside the status;
    ``output`` is None when the action commands nothing."""
    def __call__(self, ctx: Any) -> TickResult: ...


class ConditionFn(Protocol):
    """`(ctx) -> bool`: the contract Condition expects."""
    def __call__(self, ctx: Any) -> bool: ...


class SwitchKeyFn(Protocol):
    """`(ctx) -> Hashable`: the contract Switch.key_fn expects."""
    def __call__(self, ctx: Any) -> Hashable: ...


class BTNode(ABC):
    """Abstract base; subclasses override tick()."""
    name: str = "node"

    @abstractmethod
    def tick(self, ctx) -> TickResult:
        ...

    def reset(self) -> None:
        """Optional hook called when the tree is preempted; default no-op."""
        return None


class Action(BTNode):
    """Wraps a callable `(ctx) -> (Status, output)`: typed via the
    ``ActionFn`` Protocol; returns the output payload alongside the status."""
    def __init__(self, fn: ActionFn, name: str = "action"):
        self.fn = fn
        self.name = name

    def tick(self, ctx) -> TickResult:
        return self.fn(ctx)


class Condition(BTNode):
    """Wraps a callable `(ctx) -> bool`: typed via the ``ConditionFn``
    Protocol. True maps to SUCCESS, False to FAILURE.
    Conditions command nothing, so they carry a None output."""
    def __init__(self, fn: ConditionFn, name: str = "cond"):
        self.fn = fn
        self.name = name

    def tick(self, ctx) -> TickResult:
        return (Status.SUCCESS if self.fn(ctx) else Status.FAILURE), None


class Sequence(BTNode):
    """Tick children left-to-right. Stop on first FAILURE/RUNNING. SUCCESS only
    if all children succeed."""
    def __init__(self, children: List[BTNode], name: str = "sequence", memory: bool = True):
        self.children = list(children)
        self.name = name
        self.memory = memory
        self._cursor = 0

    def tick(self, ctx) -> TickResult:
        start = self._cursor if self.memory else 0
        out = None
        for i in range(start, len(self.children)):
            s, out = self.children[i].tick(ctx)
            if s == Status.FAILURE:
                self._cursor = 0
                return Status.FAILURE, out
            if s == Status.RUNNING:
                self._cursor = i if self.memory else 0
                return Status.RUNNING, out
        self._cursor = 0
        return Status.SUCCESS, out

    def reset(self) -> None:
        self._cursor = 0
        for c in self.children:
            c.reset()


class Selector(BTNode):
    """Tick children left-to-right. Stop on first SUCCESS/RUNNING. FAILURE only
    if all children fail."""
    def __init__(self, children: List[BTNode], name: str = "selector"):
        self.children = list(children)
        self.name = name

    def tick(self, ctx) -> TickResult:
        for c in self.children:
            s, out = c.tick(ctx)
            if s in (Status.SUCCESS, Status.RUNNING):
                return s, out
        return Status.FAILURE, None

    def reset(self) -> None:
        for c in self.children:
            c.reset()


class Switch(BTNode):
    """Routes to a single child based on a key returned by `key_fn(ctx)`
    (typed via ``SwitchKeyFn`` Protocol).
    Children are stored in a dict keyed by the same value. Useful for
    'switch on TaskType' selectors."""
    def __init__(self, key_fn: SwitchKeyFn, branches: dict, default: Optional[BTNode] = None,
                 name: str = "switch"):
        self.key_fn = key_fn
        self.branches = dict(branches)
        self.default = default
        self.name = name
        self._last_key = None

    def tick(self, ctx) -> TickResult:
        key = self.key_fn(ctx)
        if key != self._last_key:
            # Reset previous branch so it cleanly preempts.
            prev = self.branches.get(self._last_key) or self.default
            if prev is not None:
                prev.reset()
            self._last_key = key
        node = self.branches.get(key, self.default)
        if node is None:
            return Status.FAILURE, None
        return node.tick(ctx)

    def reset(self) -> None:
        self._last_key = None
        for c in self.branches.values():
            c.reset()
        if self.default is not None:
            self.default.reset()
