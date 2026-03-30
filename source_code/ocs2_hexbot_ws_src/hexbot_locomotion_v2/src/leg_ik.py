from __future__ import annotations

from dataclasses import dataclass
import math


def vec_sub(left: list[float], right: list[float]) -> list[float]:
    return [left[idx] - right[idx] for idx in range(3)]


def vec_norm(vector: list[float]) -> float:
    return math.sqrt(sum(value * value for value in vector))


def mat3_transpose(matrix: list[list[float]]) -> list[list[float]]:
    return [[matrix[col][row] for col in range(3)] for row in range(3)]


def mat3_mul(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [
        [sum(left[row][idx] * right[idx][col] for idx in range(3)) for col in range(3)]
        for row in range(3)
    ]


def mat3_vec_mul(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3)]


def solve_linear_3x3(matrix: list[list[float]], rhs: list[float]) -> list[float]:
    augmented = [matrix[row][:] + [rhs[row]] for row in range(3)]

    for pivot in range(3):
        best_row = max(range(pivot, 3), key=lambda row: abs(augmented[row][pivot]))
        if abs(augmented[best_row][pivot]) < 1.0e-12:
            raise ValueError("Singular 3x3 system encountered in IK solver.")
        if best_row != pivot:
            augmented[pivot], augmented[best_row] = augmented[best_row], augmented[pivot]

        pivot_value = augmented[pivot][pivot]
        for col in range(pivot, 4):
            augmented[pivot][col] /= pivot_value

        for row in range(3):
            if row == pivot:
                continue
            factor = augmented[row][pivot]
            for col in range(pivot, 4):
                augmented[row][col] -= factor * augmented[pivot][col]

    return [augmented[row][3] for row in range(3)]


@dataclass
class IKSolution:
    joint_angles: list[float]
    success: bool
    iterations: int
    error_norm: float


class LegIKSolver:
    def __init__(
        self,
        max_iterations: int = 60,
        tolerance: float = 1.0e-4,
        damping: float = 5.0e-3,
        finite_difference_step: float = 1.0e-5,
        max_delta_per_iteration: float = 0.25,
        joint_lower_limit: float = -3.14,
        joint_upper_limit: float = 3.14,
        joint_lower_limits: list[float] | None = None,
        joint_upper_limits: list[float] | None = None,
    ) -> None:
        self.max_iterations = int(max_iterations)
        self.tolerance = float(tolerance)
        self.damping = float(damping)
        self.finite_difference_step = float(finite_difference_step)
        self.max_delta_per_iteration = float(max_delta_per_iteration)
        self.joint_lower_limit = float(joint_lower_limit)
        self.joint_upper_limit = float(joint_upper_limit)
        self.joint_lower_limits = self._normalize_joint_limits(
            joint_lower_limits, self.joint_lower_limit
        )
        self.joint_upper_limits = self._normalize_joint_limits(
            joint_upper_limits, self.joint_upper_limit
        )

    @classmethod
    def from_config(cls, ik_config: dict[str, float]) -> "LegIKSolver":
        return cls(**ik_config)

    def _normalize_joint_limits(
        self,
        joint_limits: list[float] | None,
        fallback_limit: float,
    ) -> list[float]:
        if joint_limits is None:
            return [float(fallback_limit)] * 3
        if len(joint_limits) != 3:
            raise ValueError(
                "Expected exactly three joint limits for coxa/femur/tarsus."
            )
        return [float(value) for value in joint_limits]

    def solve(
        self,
        geometry,
        leg_name: str,
        target_position: list[float],
        initial_guess: list[float] | None = None,
    ) -> IKSolution:
        if initial_guess is None:
            joint_angles = list(geometry.neutral_joint_positions[leg_name])
        else:
            joint_angles = [float(value) for value in initial_guess]

        target_position = [float(value) for value in target_position]
        best_angles = joint_angles.copy()
        best_error_norm = float("inf")

        for iteration in range(1, self.max_iterations + 1):
            current_position = geometry.forward_leg(leg_name, joint_angles)
            error = vec_sub(target_position, current_position)
            error_norm = vec_norm(error)

            if error_norm < best_error_norm:
                best_error_norm = error_norm
                best_angles = joint_angles.copy()

            if error_norm <= self.tolerance:
                return IKSolution(joint_angles, True, iteration, error_norm)

            jacobian = self._finite_difference_jacobian(geometry, leg_name, joint_angles)
            jacobian_t = mat3_transpose(jacobian)
            jj_t = mat3_mul(jacobian, jacobian_t)
            damping_matrix = [
                [self.damping**2 if row == col else 0.0 for col in range(3)]
                for row in range(3)
            ]
            lhs = [
                [jj_t[row][col] + damping_matrix[row][col] for col in range(3)]
                for row in range(3)
            ]
            try:
                intermediate = solve_linear_3x3(lhs, error)
            except ValueError:
                return IKSolution(best_angles, False, iteration, best_error_norm)
            delta = mat3_vec_mul(jacobian_t, intermediate)
            delta = [
                max(-self.max_delta_per_iteration, min(self.max_delta_per_iteration, value))
                for value in delta
            ]

            joint_angles = [joint_angles[idx] + delta[idx] for idx in range(3)]
            joint_angles = [
                max(
                    self.joint_lower_limits[idx],
                    min(self.joint_upper_limits[idx], value),
                )
                for idx, value in enumerate(joint_angles)
            ]

        return IKSolution(best_angles, False, self.max_iterations, best_error_norm)

    def _finite_difference_jacobian(
        self, geometry, leg_name: str, joint_angles: list[float]
    ) -> list[list[float]]:
        jacobian = [[0.0, 0.0, 0.0] for _ in range(3)]
        for joint_index in range(3):
            positive = joint_angles.copy()
            negative = joint_angles.copy()
            positive[joint_index] += self.finite_difference_step
            negative[joint_index] -= self.finite_difference_step

            position_positive = geometry.forward_leg(leg_name, positive)
            position_negative = geometry.forward_leg(leg_name, negative)
            for axis in range(3):
                jacobian[axis][joint_index] = (
                    position_positive[axis] - position_negative[axis]
                ) / (2.0 * self.finite_difference_step)

        return jacobian
