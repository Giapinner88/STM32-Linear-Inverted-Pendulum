"""Real-time supervisor for the NMPC.

NMPC runs while it keeps its 5 ms frame. When it stops doing so (solves running
long, telemetry backing up, QP failing) control drops to the LQR + energy
shaping law, and returns to NMPC only after it has proven it is fast again.
The board's own 30 ms command timeout is the last line behind this one.
"""

from __future__ import annotations

from collections import deque


class RealtimeGuard:
    def __init__(self, deadline_ms: float = 4.5, window: int = 20, trip: int = 5,
                 severe_ms: float = 12.0, min_fallback_frames: int = 400,
                 trial_every: int = 20, trials_needed: int = 5) -> None:
        self.deadline_ms = deadline_ms
        self.window = deque(maxlen=window)
        self.trip = trip
        self.severe_ms = severe_ms
        self.min_fallback_frames = min_fallback_frames
        self.trial_every = trial_every
        self.trials_needed = trials_needed
        self.fallback = False
        self.frames_in_fallback = 0
        self.good_trials = 0
        self.events: list[tuple[str, int, str]] = []
        self.frame = 0

    def record(self, compute_ms: float, backlog: bool, qp_ok: bool) -> None:
        """Call once per frame while NMPC is in control."""
        self.frame += 1
        late = compute_ms > self.deadline_ms or backlog or not qp_ok
        # A single multi-frame stall (OS hiccup) is not proof that NMPC cannot
        # keep up: it weighs as three late frames and must be joined by more.
        weight = 3 if compute_ms > self.severe_ms else int(late)
        self.window.append(weight)
        if sum(self.window) >= self.trip:
            reason = (f"{sum(self.window)} lateness points in {len(self.window)} frames "
                      f"(last solve {compute_ms:.1f} ms" + (", QP failure" if not qp_ok else "") + ")")
            self.fallback = True
            self.frames_in_fallback = 0
            self.good_trials = 0
            self.window.clear()
            self.events.append(("fallback", self.frame, reason))

    def tick_fallback(self) -> bool:
        """Call once per frame while in fallback; True if a trial solve is due."""
        self.frame += 1
        self.frames_in_fallback += 1
        return (self.frames_in_fallback >= self.min_fallback_frames
                and self.frames_in_fallback % self.trial_every == 0)

    def record_trial(self, solve_ms: float, backlog: bool) -> bool:
        """Result of a trial solve; True when NMPC may take over again."""
        if solve_ms <= self.deadline_ms and not backlog:
            self.good_trials += 1
        else:
            self.good_trials = 0
        if self.good_trials >= self.trials_needed:
            self.fallback = False
            self.window.clear()
            self.events.append(("nmpc", self.frame, f"{self.good_trials} fast trials"))
            return True
        return False
