"""将已检查的关节路径转换成限速、限加速度的同步五次轨迹。

每个路点停至零速度和零加速度，段内所有关节使用同一单调插值参数，
因此仍沿规划器检查过的关节直线路径行进。这里只输出参考值，不连接驱动。
"""

from dataclasses import dataclass

import numpy as np

from .validation import finite_matrix, finite_vector


@dataclass(frozen=True)
class TrajectorySample:
    """某个时刻的参考关节变量、速度、加速度。

    hinge 分别为 rad、rad/s、rad/s²；slide 分别为 m、m/s、m/s²。
    Example Input / Output:
        >>> sample = TrajectorySample(np.array([0.2]), np.array([0.0]), np.array([0.0]))
        >>> sample.joint_variables.tolist()
        [0.2]
    """

    joint_variables: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray


class JointTrajectory:
    def __init__(self, waypoints, velocity_limits, acceleration_limits, minimum_segment_time: float = 0.05):
        """根据每关节速度和加速度限值分配各段时间，不改变路径几何。

        :param waypoints: (路点数, DOF) 数组；hinge/rad，slide/m。
        :param velocity_limits: 每个关节的最大速度，分别为 rad/s 或 m/s。
        :param acceleration_limits: 每个关节最大加速度，rad/s² 或 m/s²。
        :param minimum_segment_time: 非零运动段的最小时长/s。
        :return: 构造函数无返回值；duration 为总时长/s。

        Example Input / Output:
            >>> trajectory = JointTrajectory([[0], [1]], [1], [10])
            >>> trajectory.duration
            1.875
        """
        points = finite_matrix(waypoints, (None, None), "waypoints")
        self.dof = points.shape[1]
        self.velocity_limits = finite_vector(velocity_limits, self.dof, "velocity limits")
        self.acceleration_limits = finite_vector(acceleration_limits, self.dof, "acceleration limits")
        if np.any(self.velocity_limits <= 0) or np.any(self.acceleration_limits <= 0):
            raise ValueError("velocity and acceleration limits must be positive")
        minimum = float(finite_vector([minimum_segment_time], 1, "minimum segment time")[0])
        if minimum <= 0:
            raise ValueError("minimum segment time must be positive")
        keep = np.r_[True, np.any(np.diff(points, axis=0) != 0, axis=1)]
        self.waypoints = points[keep]
        changes = np.abs(np.diff(self.waypoints, axis=0))
        durations = []
        for delta in changes:
            velocity_time = np.max(1.875 * delta / self.velocity_limits)
            acceleration_time = np.max(np.sqrt((10 * np.sqrt(3) / 3) * delta / self.acceleration_limits))
            durations.append(max(minimum, float(velocity_time), float(acceleration_time)))
        self.segment_times = np.asarray(durations)
        self.times = np.r_[0.0, np.cumsum(self.segment_times)]
        self.duration = float(self.times[-1])
        for value in (self.waypoints, self.velocity_limits, self.acceleration_limits, self.segment_times, self.times):
            value.setflags(write=False)

    def sample(self, elapsed_time: float) -> TrajectorySample:
        """计算任意时刻参考值；末尾返回目标位置和零速度/加速度。

        使用 s(u)=10u³-15u⁴+6u⁵。其最大一阶导数为 1.875，
        最大二阶导数绝对值为 10*sqrt(3)/3，已用于分配满足限值的时长。

        :param elapsed_time: 从轨迹起始时刻累计的时间/s。
        :return: TrajectorySample，数组为独立副本。

        Example Input / Output:
            >>> trajectory = JointTrajectory([[0], [1]], [1], [10])
            >>> trajectory.sample(trajectory.duration / 2).joint_variables.tolist()
            [0.5]
            >>> trajectory.sample(trajectory.duration).velocity.tolist()
            [0.0]
        """
        time = float(finite_vector([elapsed_time], 1, "elapsed time")[0])
        if time <= 0:
            return TrajectorySample(self.waypoints[0].copy(), np.zeros(self.dof), np.zeros(self.dof))
        if time >= self.duration:
            return TrajectorySample(self.waypoints[-1].copy(), np.zeros(self.dof), np.zeros(self.dof))
        index = int(np.searchsorted(self.times, time, side="right") - 1)
        duration = self.segment_times[index]
        u = (time - self.times[index]) / duration
        fraction = 10 * u**3 - 15 * u**4 + 6 * u**5
        speed = (30 * u**2 - 60 * u**3 + 30 * u**4) / duration
        acceleration = (60 * u - 180 * u**2 + 120 * u**3) / duration**2
        delta = self.waypoints[index + 1] - self.waypoints[index]
        return TrajectorySample(self.waypoints[index] + fraction * delta, speed * delta, acceleration * delta)

    def iter_samples(self, period: float):
        """按指定周期逐项生成带时间戳的参考值，避免一次分配大数组。

        :param period: 控制周期/s，必须是有限正数。
        :return: 迭代器，每项 (累计时间/s, TrajectorySample)，包含最终目标。

        Example Input / Output:
            >>> trajectory = JointTrajectory([[0]], [1], [1])
            >>> [(time, sample.joint_variables.tolist()) for time, sample in trajectory.iter_samples(0.01)]
            [(0.0, [0.0])]
        """
        period = float(finite_vector([period], 1, "sample period")[0])
        if period <= 0:
            raise ValueError("sample period must be positive")
        count = int(np.ceil(self.duration / period))
        for index in range(count + 1):
            time = min(index * period, self.duration)
            yield time, self.sample(time)
