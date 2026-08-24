#!/usr/bin/env python3
"""Detect a stalled-but-"solved" curobo execution and decide when to resync.

Symptom this guards against (not something this class can fix): curobo's
reactive MPC controller integrates its own commanded state rather than the
real robot state. A single rejected SpeedJ command (e.g. an acceleration
limit violation, or a missed control-loop deadline) then leaves the real arm
stalled while the solver's internal plan keeps "converging" on a state the
arm never reached — `controller_position_error` heads to ~0 while the
FK-measured `position_error` plateaus. `on_target` never latches, and no
further motion happens: the topic-level (`mpc_goal`) retargeting path uses
the same optimizer state, so it doesn't recover on its own. Cancelling and
resending the goal makes the server re-seed its plan from the actual robot
state, which does recover it (observed operationally: this is the "cancel
and retrigger" workaround that reliably unsticks it).

`ExecutionDivergenceWatchdog` only detects the pattern and rate-limits the
recovery action; it doesn't perform the cancel/resend itself. That stays in
grasp.py, since it owns the action client and the state machine.
"""


class ExecutionDivergenceWatchdog:
    """Flags when FK-measured error has stopped tracking the solver's own
    error estimate, for long enough, to be worth a resync — and rate-limits
    how often that resync may fire.

    One instance covers a single execute_trajectory attempt (including any
    resyncs within it): call `reset()` at the start of each new attempt
    (i.e. each `act_move_pregrasp`/`act_move_grasp` invocation), not once
    per resync.
    """

    # controller_position_error sentinel meaning "active planner doesn't
    # expose it" (see curobo_msgs/action/SendTrajectory.action) — only LBFGS
    # currently fills it in.
    _NOT_EXPOSED = -1.0

    def __init__(self, divergence_threshold=0.02, debounce_count=3,
                 grace_period_s=3.0, max_resyncs=3, min_resync_interval_s=5.0):
        """
        Args:
            divergence_threshold: meters. Resync candidate once
                position_error - controller_position_error exceeds this —
                i.e. the solver believes it's this much closer than the real
                arm is.
            debounce_count: consecutive over-threshold feedback samples
                required before triggering, so one noisy sample can't fire it.
            grace_period_s: seconds after a resync during which divergence is
                ignored. A fresh goal legitimately starts with a large,
                shrinking position_error; without this the watchdog would
                immediately re-trigger on its own resync.
            max_resyncs: resyncs allowed per attempt (per `reset()`). Once
                exhausted, on_feedback always returns False — the caller
                should treat the attempt as failed rather than retry forever.
            min_resync_interval_s: minimum wall-clock time between two
                resyncs, independent of the grace period, as a second guard
                against thrashing if grace_period_s is set very low.
        """
        self.divergence_threshold = divergence_threshold
        self.debounce_count = debounce_count
        self.grace_period_s = grace_period_s
        self.max_resyncs = max_resyncs
        self.min_resync_interval_s = min_resync_interval_s
        self.reset()

    def reset(self):
        """Start tracking a fresh execute_trajectory attempt."""
        self._streak = 0
        self._resync_count = 0
        self._last_resync_time_s = None

    # on_feedback() return values.
    OK = 'ok'                          # no action
    RESYNC = 'resync'                  # caller should cancel + resend now
    BUDGET_EXHAUSTED = 'exhausted'     # still diverging, but out of resyncs —
                                        # caller should fail the attempt

    @property
    def resync_count(self):
        return self._resync_count

    @property
    def budget_exhausted(self):
        return self._resync_count >= self.max_resyncs

    def on_feedback(self, position_error, controller_position_error, now_s):
        """Feed one action-feedback sample.

        Args:
            position_error: fb.position_error (FK-measured, meters)
            controller_position_error: fb.controller_position_error (meters,
                or the -1.0 "not exposed" sentinel)
            now_s: current time in seconds (any monotonic-ish clock, as long
                as it's consistent across calls on this instance)

        Returns:
            One of OK, RESYNC, BUDGET_EXHAUSTED. RESYNC is only returned once
            per debounced divergence episode (and already counts against the
            budget); BUDGET_EXHAUSTED is returned on every debounced episode
            once the budget is used up, so the caller can fail fast rather
            than let the attempt hang forever like the un-patched behavior.
        """
        if controller_position_error == self._NOT_EXPOSED:
            self._streak = 0
            return self.OK

        if (self._last_resync_time_s is not None
                and (now_s - self._last_resync_time_s) < self.grace_period_s):
            self._streak = 0
            return self.OK

        divergence = position_error - controller_position_error
        if divergence <= self.divergence_threshold:
            self._streak = 0
            return self.OK

        self._streak += 1
        if self._streak < self.debounce_count:
            return self.OK
        self._streak = 0

        if self.budget_exhausted:
            return self.BUDGET_EXHAUSTED
        if (self._last_resync_time_s is not None
                and (now_s - self._last_resync_time_s) < self.min_resync_interval_s):
            return self.OK

        self._resync_count += 1
        self._last_resync_time_s = now_s
        return self.RESYNC
