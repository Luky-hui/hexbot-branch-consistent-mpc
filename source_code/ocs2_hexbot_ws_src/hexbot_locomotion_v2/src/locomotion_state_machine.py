from __future__ import annotations


WAIT_FOR_VALID_SEED = "WAIT_FOR_VALID_SEED"
RECOVER_TO_STAND = "RECOVER_TO_STAND"
READY = "READY"
WALKING = "WALKING"
STOPPING_TO_STAND = "STOPPING_TO_STAND"
FAULT = "FAULT"


class LocomotionStateMachine:
    def __init__(self, initial_state: str = WAIT_FOR_VALID_SEED) -> None:
        self.state = str(initial_state)
        self.seed_source: str | None = None
        self.fault_reason: str | None = None

    def set_state(self, state: str) -> None:
        self.state = str(state)
        if self.state != FAULT:
            self.fault_reason = None

    def set_ready(self, seed_source: str) -> None:
        self.seed_source = str(seed_source)
        self.set_state(READY)

    def begin_recovery(self) -> None:
        self.set_state(RECOVER_TO_STAND)

    def begin_walking(self) -> None:
        self.set_state(WALKING)

    def begin_stopping(self) -> None:
        self.set_state(STOPPING_TO_STAND)

    def enter_fault(self, reason: str) -> None:
        self.fault_reason = str(reason)
        self.state = FAULT

    def is_fault(self) -> bool:
        return self.state == FAULT
