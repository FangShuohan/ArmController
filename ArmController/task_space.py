"""从螺旋轴构造固定任务子空间；残差行数与关节数独立。"""

import numpy as np

from .geometry import Pose, pose_error


def _basis(vectors):
    """求三维方向集合的正交基，先归一化避免米与角度的尺度影响秩。"""
    columns = [v / np.linalg.norm(v) for v in vectors if np.linalg.norm(v) > 1e-12]
    if not columns:
        return np.empty((3, 0))
    u, singular, _ = np.linalg.svd(np.column_stack(columns), full_matrices=False)
    return u[:, singular > 1e-10]


def _closure(seeds, angular_axes):
    """闭包保证方向空间在各关节旋转下不变，至多扩展到三维。"""
    basis = _basis(seeds)
    for _ in range(3):
        expanded = _basis([*basis.T, *(np.cross(w, v) for w in angular_axes for v in basis.T)])
        if expanded.shape[1] == basis.shape[1]:
            return expanded
        basis = expanded
    return basis


class TaskSpace:
    """运动学模型可变化的位置/姿态子空间，保守地保留可能变化的方向。

    auto 压缩模型恒定的方向；full 保留六维；position 只约束位置。
    不按单个构型的雅可比秩截断，所以零位奇异构型不会丢失任务方向。
    本子空间维数是任务残差行数，并不是瞬时雅可比秩或机械臂 DOF。
    """

    def __init__(self, kinematics, mode="auto"):
        if mode not in ("auto", "full", "position"):
            raise ValueError("task_mode must be auto, full or position")
        self.mode = mode
        self.zero_pose = Pose.from_matrix(kinematics.zero_transform)
        angular = kinematics.screw_axes[:3].T
        linear = kinematics.screw_axes[3:].T
        position_seeds = [np.cross(w, self.zero_pose.position) + v for w, v in zip(angular, linear)]
        self.position_basis = _closure(position_seeds, angular)
        self.orientation_basis = _closure(list(angular), angular)
        if mode == "full":
            self.position_basis = np.eye(3)
            self.orientation_basis = np.eye(3)
        elif mode == "position":
            self.orientation_basis = np.empty((3, 0))
        self.residual_dimension = self.position_basis.shape[1] + self.orientation_basis.shape[1]
        self.position_basis.setflags(write=False)
        self.orientation_basis.setflags(write=False)

    def residual(self, actual, target, orientation_weight):
        position, orientation = pose_error(actual, target)
        return np.r_[self.position_basis.T @ position,
                     orientation_weight * (self.orientation_basis.T @ orientation)]

    def require_compatible(self, target, position_tolerance, orientation_tolerance):
        """拒绝模型恒定方向上的不可满足目标；最终仍检查完整位姿误差。"""
        position, orientation = pose_error(self.zero_pose, target)
        outside_position = position - self.position_basis @ (self.position_basis.T @ position)
        if np.linalg.norm(outside_position) > position_tolerance:
            raise ValueError("target position violates model-invariant directions")
        if self.mode != "position":
            outside_orientation = orientation - self.orientation_basis @ (self.orientation_basis.T @ orientation)
            if np.linalg.norm(outside_orientation) > orientation_tolerance:
                raise ValueError("target orientation violates model-invariant directions")
