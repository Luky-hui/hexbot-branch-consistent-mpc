from __future__ import annotations

from dataclasses import dataclass
import math


def clamp(value: float, lower: float, upper: float) -> float:
    if lower > upper:
        lower, upper = upper, lower
    return max(lower, min(upper, value))


def low_pass(previous: float, current: float, alpha: float) -> float:
    alpha = clamp(alpha, 0.0, 1.0)
    return previous + alpha * (current - previous)


def quaternion_to_roll_pitch(quaternion: tuple[float, float, float, float]) -> tuple[float, float]:
    x_value, y_value, z_value, w_value = quaternion

    sinr_cosp = 2.0 * (w_value * x_value + y_value * z_value)
    cosr_cosp = 1.0 - 2.0 * (x_value * x_value + y_value * y_value)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w_value * y_value - z_value * x_value)
    pitch = math.asin(clamp(sinp, -1.0, 1.0))
    return roll, pitch


@dataclass
class ImuEstimate:
    roll: float = 0.0
    pitch: float = 0.0
    roll_rate: float = 0.0
    pitch_rate: float = 0.0
    healthy: bool = False
    status: str = "waiting_for_imu"
    last_update_time_sec: float | None = None


class ImuStateEstimator:
    def __init__(
        self,
        imu_timeout_sec: float,
        orientation_alpha: float,
        angular_velocity_alpha: float,
        roll_offset_rad: float,
        pitch_offset_rad: float,
        gravity_min_mps2: float,
        gravity_max_mps2: float,
        fail_open: bool,
    ) -> None:
        self.imu_timeout_sec = max(0.01, float(imu_timeout_sec))
        self.orientation_alpha = clamp(float(orientation_alpha), 0.0, 1.0)
        self.angular_velocity_alpha = clamp(float(angular_velocity_alpha), 0.0, 1.0)
        self.roll_offset_rad = float(roll_offset_rad)
        self.pitch_offset_rad = float(pitch_offset_rad)
        self.gravity_min_mps2 = max(0.0, float(gravity_min_mps2))
        self.gravity_max_mps2 = max(self.gravity_min_mps2, float(gravity_max_mps2))
        self.fail_open = bool(fail_open)
        self.estimate = ImuEstimate()

    def update_from_message(self, message, now_sec: float) -> ImuEstimate:
        quaternion = (
            float(message.orientation.x),
            float(message.orientation.y),
            float(message.orientation.z),
            float(message.orientation.w),
        )
        if not all(math.isfinite(value) for value in quaternion):
            self.estimate.healthy = False
            self.estimate.status = "imu_quaternion_non_finite"
            return self.current(now_sec)

        quat_norm = math.sqrt(sum(value * value for value in quaternion))
        if quat_norm <= 1.0e-6:
            self.estimate.healthy = False
            self.estimate.status = "imu_quaternion_zero_norm"
            return self.current(now_sec)

        quaternion = tuple(value / quat_norm for value in quaternion)
        roll, pitch = quaternion_to_roll_pitch(quaternion)
        roll -= self.roll_offset_rad
        pitch -= self.pitch_offset_rad

        acceleration = (
            float(message.linear_acceleration.x),
            float(message.linear_acceleration.y),
            float(message.linear_acceleration.z),
        )
        if not all(math.isfinite(value) for value in acceleration):
            self.estimate.healthy = False
            self.estimate.status = "imu_acceleration_non_finite"
            return self.current(now_sec)

        gravity_norm = math.sqrt(sum(value * value for value in acceleration))
        if gravity_norm < self.gravity_min_mps2 or gravity_norm > self.gravity_max_mps2:
            self.estimate.healthy = False
            self.estimate.status = "imu_gravity_out_of_range"
            return self.current(now_sec)

        roll_rate = float(message.angular_velocity.x)
        pitch_rate = float(message.angular_velocity.y)
        if not math.isfinite(roll_rate) or not math.isfinite(pitch_rate):
            self.estimate.healthy = False
            self.estimate.status = "imu_angular_velocity_non_finite"
            return self.current(now_sec)

        previous = self.estimate
        if previous.last_update_time_sec is None:
            filtered_roll = roll
            filtered_pitch = pitch
            filtered_roll_rate = roll_rate
            filtered_pitch_rate = pitch_rate
        else:
            filtered_roll = low_pass(previous.roll, roll, self.orientation_alpha)
            filtered_pitch = low_pass(previous.pitch, pitch, self.orientation_alpha)
            filtered_roll_rate = low_pass(previous.roll_rate, roll_rate, self.angular_velocity_alpha)
            filtered_pitch_rate = low_pass(previous.pitch_rate, pitch_rate, self.angular_velocity_alpha)

        self.estimate = ImuEstimate(
            roll=filtered_roll,
            pitch=filtered_pitch,
            roll_rate=filtered_roll_rate,
            pitch_rate=filtered_pitch_rate,
            healthy=True,
            status="imu_ok",
            last_update_time_sec=float(now_sec),
        )
        return self.estimate

    def current(self, now_sec: float) -> ImuEstimate:
        if self.estimate.last_update_time_sec is None:
            return ImuEstimate(status="waiting_for_imu")

        age_sec = float(now_sec) - float(self.estimate.last_update_time_sec)
        if age_sec > self.imu_timeout_sec:
            return ImuEstimate(
                roll=self.estimate.roll,
                pitch=self.estimate.pitch,
                roll_rate=self.estimate.roll_rate,
                pitch_rate=self.estimate.pitch_rate,
                healthy=False,
                status="imu_timeout",
                last_update_time_sec=self.estimate.last_update_time_sec,
            )
        return self.estimate
