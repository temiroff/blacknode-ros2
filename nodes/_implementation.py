"""Keep node contracts beside implementations until a component activates them."""
from __future__ import annotations

from collections.abc import Callable
from typing import Any

from blacknode.node import node as runtime_node

_SPEC_ATTR = "_bn_component_node_spec"


def implementation_node(**spec: Any) -> Callable[[Callable], Callable]:
    """Record a node decorator without registering it from the core module."""

    def decorator(fn: Callable) -> Callable:
        setattr(fn, _SPEC_ATTR, dict(spec))
        return fn

    return decorator


def register_implementation(fn: Callable) -> Callable:
    """Register one implementation from its enabled component entry module."""
    spec = getattr(fn, _SPEC_ATTR, None)
    if not isinstance(spec, dict):
        raise TypeError(f"{fn.__module__}.{fn.__name__} has no component node contract")
    return runtime_node(**spec)(fn)
