from __future__ import annotations


class FootTrajectoryGenerator:
    def __init__(self, lift_height: float = 0.02) -> None:
        self.lift_height = float(lift_height)

    def swing_point(self, start: list[float], end: list[float], phase: float) -> list[float]:
        phase = max(0.0, min(1.0, float(phase)))
        point = [
            float(start[idx]) + phase * (float(end[idx]) - float(start[idx]))
            for idx in range(3)
        ]
        point[2] += 4.0 * self.lift_height * phase * (1.0 - phase)
        return point

    def stance_point(self, start: list[float], end: list[float], phase: float) -> list[float]:
        phase = max(0.0, min(1.0, float(phase)))
        return [
            float(start[idx]) + phase * (float(end[idx]) - float(start[idx]))
            for idx in range(3)
        ]

    def single_leg_cycle(
        self, home: list[float], offset: list[float], phase: float
    ) -> list[float]:
        home = [float(value) for value in home]
        offset = [float(value) for value in offset]
        target = [home[idx] + offset[idx] for idx in range(3)]

        if phase < 0.5:
            return self.swing_point(home, target, phase / 0.5)
        return self.stance_point(target, home, (phase - 0.5) / 0.5)

    def tripod_step(
        self,
        home: list[float],
        step_delta: list[float],
        phase: float,
        is_swing: bool,
    ) -> list[float]:
        home = [float(value) for value in home]
        step_delta = [float(value) for value in step_delta]
        front = [home[idx] + step_delta[idx] for idx in range(3)]
        rear = [home[idx] - step_delta[idx] for idx in range(3)]
        front[2] = home[2]
        rear[2] = home[2]

        if is_swing:
            return self.swing_point(rear, front, phase)
        return self.stance_point(front, rear, phase)
