from __future__ import annotations

from dataclasses import dataclass


@dataclass
class TripodState:
    phase: float
    swing_legs: tuple[str, ...]
    stance_legs: tuple[str, ...]


class TripodScheduler:
    def __init__(self, tripod_groups: dict[str, list[str]], cycle_length: int = 8) -> None:
        self.tripod_a = tuple(tripod_groups["A"])
        self.tripod_b = tuple(tripod_groups["B"])
        self.cycle_length = int(cycle_length)
        self.phase = 0.0

    def advance(self, dt: float, cadence_hz: float = 1.0) -> TripodState:
        self.phase = (self.phase + max(dt, 0.0) * max(cadence_hz, 0.0)) % 1.0
        return self.state()

    def state(self) -> TripodState:
        if self.phase < 0.5:
            return TripodState(self.phase, self.tripod_a, self.tripod_b)
        return TripodState(self.phase, self.tripod_b, self.tripod_a)

    def leg_state(self, leg_name: str) -> tuple[bool, float]:
        if leg_name in self.tripod_a:
            if self.phase < 0.5:
                return True, self.phase / 0.5
            return False, (self.phase - 0.5) / 0.5

        if self.phase < 0.5:
            return False, self.phase / 0.5
        return True, (self.phase - 0.5) / 0.5
