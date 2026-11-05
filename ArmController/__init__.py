# ruff: noqa: N999 -- 保留项目已有的 ArmController 目录名
"""串联机械臂纯算法：位姿、运动学、自碰撞、路径与轨迹。"""

from .collision import Capsule, SelfCollisionModel
from .coordination import PhaseSegment
from .geometry import Pose
from .inverse_kinematics import IKSettings, IKSolution, IKSolver
from .kinematics import POEKinematics
from .path_planning import JointPathPlanner, PlannerSettings, PlanningError
from .task_space import TaskSpace
from .trajectory import JointTrajectory, TrajectorySample

__all__ = [
    "Capsule", "IKSettings", "IKSolution", "IKSolver", "JointPathPlanner", "JointTrajectory",
    "POEKinematics", "PlannerSettings", "PlanningError", "Pose", "SelfCollisionModel",
    "TaskSpace", "TrajectorySample", "PhaseSegment",
]
