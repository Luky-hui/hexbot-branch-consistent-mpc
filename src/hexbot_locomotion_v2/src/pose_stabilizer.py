from __future__ import annotations

from dataclasses import dataclass


def clamp(value: float, lower: float, upper: float) -> float:
    if lower > upper:
        lower, upper = upper, lower
    return max(lower, min(upper, value))


@dataclass
class StabilizerCorrection:
    roll_correction: float = 0.0
    pitch_correction: float = 0.0
    active: bool = False


class PoseStabilizer:
    def __init__(
        self,
        enabled: bool,
        deadband_rad: float,
        roll_kp: float,
        roll_kd: float,
        pitch_kp: float,
        pitch_kd: float,
        max_roll_correction_rad: float,
        max_pitch_correction_rad: float,
        max_correction_rate_rad_s: float,
    ) -> None:
        self.enabled = bool(enabled)
        self.deadband_rad = max(0.0, float(deadband_rad))
        self.roll_kp = float(roll_kp)
        self.roll_kd = float(roll_kd)
        self.pitch_kp = float(pitch_kp)
        self.pitch_kd = float(pitch_kd)
        self.max_roll_correction_rad = max(0.0, float(max_roll_correction_rad))
        self.max_pitch_correction_rad = max(0.0, float(max_pitch_correction_rad))
        self.max_correction_rate_rad_s = max(0.0, float(max_correction_rate_rad_s))
        self.last_roll_correction = 0.0
        self.last_pitch_correction = 0.0

    def reset(self) -> None:
        self.last_roll_correction = 0.0
        self.last_pitch_correction = 0.0

    def update(
        self,
        roll: float,
        pitch: float,
        roll_rate: float,
        pitch_rate: float,
        healthy: bool,
        dt: float,
        control_scale: float = 1.0,
    ) -> StabilizerCorrection:
        dt = max(0.0, float(dt))
        control_scale = clamp(float(control_scale), 0.0, 1.0)
        if not self.enabled or not healthy or control_scale <= 0.0:
            target_roll = 0.0
            target_pitch = 0.0
        else:
            roll_error = 0.0 if abs(roll) < self.deadband_rad else roll
            pitch_error = 0.0 if abs(pitch) < self.deadband_rad else pitch
            target_roll = control_scale * (
                -self.roll_kp * roll_error - self.roll_kd * roll_rate
            )
            target_pitch = control_scale * (
                -self.pitch_kp * pitch_error - self.pitch_kd * pitch_rate
            )

        target_roll = clamp(
            target_roll,
            -self.max_roll_correction_rad,
            self.max_roll_correction_rad,
        )
        target_pitch = clamp(
            target_pitch,
            -self.max_pitch_correction_rad,
            self.max_pitch_correction_rad,
        )

        self.last_roll_correction = self._rate_limit(
            self.last_roll_correction, target_roll, dt
        )
        self.last_pitch_correction = self._rate_limit(
            self.last_pitch_correction, target_pitch, dt
        )

        return StabilizerCorrection(
            roll_correction=self.last_roll_correction,
            pitch_correction=self.last_pitch_correction,
            active=self.enabled and healthy and control_scale > 0.0,
        )

    def _rate_limit(self, previous: float, target: float, dt: float) -> float:
        if self.max_correction_rate_rad_s <= 0.0 or dt <= 0.0:
            return float(target)
        max_delta = self.max_correction_rate_rad_s * dt
        return clamp(float(target), float(previous) - max_delta, float(previous) + max_delta)

    def apply_to_target(
        self,
        target: list[float],
        correction: StabilizerCorrection,
        weight: float = 1.0,
    ) -> list[float]:
        if weight <= 0.0:
            return [float(value) for value in target]

        adjusted = [float(value) for value in target]
        weighted_roll = correction.roll_correction * float(weight)
        weighted_pitch = correction.pitch_correction * float(weight)
        adjusted[2] += -adjusted[1] * weighted_roll + adjusted[0] * weighted_pitch
        return adjusted
