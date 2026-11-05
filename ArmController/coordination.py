"""共同进度的几何检查；不生成时间轨迹，不依赖电机或通信。"""
from dataclasses import dataclass

import numpy as np

from .validation import finite_vector


@dataclass(frozen=True)
class PhaseSegment:
    """一段直线路径 q=start+phase*(goal-start)，所有轴共用 phase∈[0,1]。

    数组单位由调用者决定，可混合 rad/m；tolerance 必须采用对应单位。
    validate 用 MCU 报告的共同进度检查反馈，避免用上位机时钟猜测速度。
    """
    start: np.ndarray
    goal: np.ndarray
    tolerance: np.ndarray

    def __post_init__(self):
        raw = np.asarray(self.start)
        if raw.ndim != 1 or not raw.size:
            raise ValueError("start must be a nonempty vector")
        for name in ("start", "goal", "tolerance"):
            value = finite_vector(getattr(self, name), raw.size, name)
            if name == "tolerance" and np.any(value <= 0):
                raise ValueError("tolerance must be positive")
            value.setflags(write=False)
            object.__setattr__(self, name, value)

    def position(self, phase):
        phase = float(finite_vector([phase], 1, "phase")[0])
        if not 0 <= phase <= 1:
            raise ValueError("phase must be in [0,1]")
        return self.start + phase * (self.goal - self.start)

    def validate(self, measured, phase):
        measured = finite_vector(measured, len(self.start), "measured position")
        if np.any(np.abs(measured - self.position(phase)) > self.tolerance):
            raise ValueError("measured axes deviate from the common-phase path")
