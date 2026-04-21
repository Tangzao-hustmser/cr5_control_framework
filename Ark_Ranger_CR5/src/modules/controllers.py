import numpy as np


class JointSmoother:
    """Exponential smoothing to reduce high-frequency command jumps."""

    def __init__(self, factor: float = 0.05):
        self.factor = float(factor)
        self.current_state = None

    def step(self, target_pos: np.ndarray) -> np.ndarray:
        target = np.asarray(target_pos, dtype=np.float64)
        if self.current_state is None:
            self.current_state = np.copy(target)
        self.current_state += self.factor * (target - self.current_state)
        return np.asarray(self.current_state, dtype=np.float64)
