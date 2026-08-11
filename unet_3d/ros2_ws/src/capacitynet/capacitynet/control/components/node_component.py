#!/usr/bin/env python3
"""Base class for ROS components grafted onto a host node.

A `NodeComponent` is *not* a `Node`: it receives one and, in its `__init__`,
declares the parameters it needs and creates its own subscriptions, publishers,
services and timers on that node. Assembling a node then means instantiating the
components you want and wiring their callbacks together — the node itself holds
only the wiring.

That is what makes the pieces testable in integration rather than in isolation:
a probe node that only wants the workspace region and the reachability inference
instantiates those two components and simply omits the one that commands the
base, so it cannot move the robot while you look at the numbers.

Parameter names are flat by default (`prefix=''`), so a component declares
exactly the names the existing launch files already pass. A prefix is only
needed when the same component type is instantiated twice on one node.
"""

from rclpy.exceptions import ParameterAlreadyDeclaredException


class NodeComponent:
    """Composant ROS greffé sur un nœud hôte : déclare ses params, crée ses endpoints."""

    def __init__(self, node, prefix='', callback_group=None):
        """Attach the component to a host node.

        Args:
            node: the host rclpy Node this component attaches to
            prefix: optional parameter/topic name prefix; '' keeps flat names
            callback_group: callback group for every endpoint this component creates
        """
        self.node = node
        self.prefix = prefix.strip('.') if prefix else ''
        self.log = node.get_logger()
        self.callback_group = callback_group

    # ── Naming ───────────────────────────────────────────────────────────────

    def _p(self, name):
        """Parameter name: 'x' -> 'prefix.x' when prefixed, else 'x'."""
        return f'{self.prefix}.{name}' if self.prefix else name

    def _private(self, name):
        """Node-private topic name: '~/x', or '~/prefix/x' when prefixed."""
        return f'~/{self.prefix}/{name}' if self.prefix else f'~/{name}'

    # ── Parameters ───────────────────────────────────────────────────────────

    def _declare(self, name, default):
        """Declare a parameter on the host node (if new) and return its value.

        Redeclaration is tolerated so that two components can share a parameter
        (e.g. `publish_debug_markers`) and so a host can pre-declare one to
        override the component's default.
        """
        full = self._p(name)
        try:
            self.node.declare_parameter(full, default)
        except ParameterAlreadyDeclaredException:
            pass
        return self.node.get_parameter(full).value

    # ── Clock helper ─────────────────────────────────────────────────────────

    def _age_s(self, stamp):
        """Seconds since `stamp`, or +inf if it was never set."""
        if stamp is None:
            return float('inf')
        return (self.node.get_clock().now() - stamp).nanoseconds * 1e-9
