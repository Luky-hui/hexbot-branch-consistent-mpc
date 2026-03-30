from __future__ import annotations

from dataclasses import dataclass


def clamp(value: float, lower: float, upper: float) -> float:
    if lower > upper:
        lower, upper = upper, lower
    return max(lower, min(upper, value))


@dataclass
class FilteredPlanarCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.vx, self.vy, self.wz)


class CommandFilter:
    def __init__(
        self,
        tau_sec: float,
        accel_limit_x: float,
        accel_limit_y: float,
        accel_limit_yaw: float,
    ) -> None:
        self.tau_sec = max(0.0, float(tau_sec))
        self.accel_limit_x = max(0.0, float(accel_limit_x))
        self.accel_limit_y = max(0.0, float(accel_limit_y))
        self.accel_limit_yaw = max(0.0, float(accel_limit_yaw))
        self.filtered = FilteredPlanarCommand()

    def reset(self) -> None:
        self.filtered = FilteredPlanarCommand()

    def step(self, target: tuple[float, float, float], dt: float) -> FilteredPlanarCommand:
        dt = max(0.0, float(dt))
        if dt <= 0.0:
            return FilteredPlanarCommand(*self.filtered.as_tuple())

        alpha = 1.0 if self.tau_sec <= 1.0e-6 else clamp(dt / (self.tau_sec + dt), 0.0, 1.0)
        smoothed = (
            self.filtered.vx + alpha * (float(target[0]) - self.filtered.vx),
            self.filtered.vy + alpha * (float(target[1]) - self.filtered.vy),
            self.filtered.wz + alpha * (float(target[2]) - self.filtered.wz),
        )

        limited = (
            self._limit_axis(self.filtered.vx, smoothed[0], self.accel_limit_x, dt),
            self._limit_axis(self.filtered.vy, smoothed[1], self.accel_limit_y, dt),
            self._limit_axis(self.filtered.wz, smoothed[2], self.accel_limit_yaw, dt),
        )
        self.filtered = FilteredPlanarCommand(*limited)
        return FilteredPlanarCommand(*limited)

    def _limit_axis(self, current: float, target: float, accel_limit: float, dt: float) -> float:
        if accel_limit <= 0.0 or dt <= 0.0:
            return float(target)
        max_delta = accel_limit * dt
        return clamp(float(target), float(current) - max_delta, float(current) + max_delta)
