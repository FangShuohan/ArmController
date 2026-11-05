"""有界 SciPy 逆解：位姿残差、初值生成、候选评分和可行性检查。

示例使用单个沿 X 轴平移的关节，便于直接看出输入输出；UR5e 运行时
传入的是 MuJoCo 提取的六个旋转关节。>>> 后是输入，下一行是输出。
"""

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from .geometry import Pose, error_norms
from .kinematics import POEKinematics
from .task_space import TaskSpace
from .validation import finite_vector

IK_MODES = ("nearest", "multi-start")


@dataclass(frozen=True)
class IKSettings:
    """逆解参数配置；位置单位 m，角度单位 rad。

    orientation_weight 控制姿态残差权重；两项 tolerance 决定可行性；
    limit_weight 和 minimum_limit_margin 控制归一化限位代价（无单位）；random_starts
    和 random_seed 控制多初值搜索；max_evaluations 和 optimizer_tolerance
    控制 SciPy 求解预算与停止条件。

    Example Input / Output:
        >>> settings = IKSettings(position_tolerance=0.001)
        >>> settings.position_tolerance, settings.orientation_tolerance
        (0.001, 0.02)
    """

    orientation_weight: float = 0.3
    position_tolerance: float = 0.005
    orientation_tolerance: float = 0.02
    limit_weight: float = 0.01
    minimum_limit_margin: float = 1e-3
    random_starts: int = 12
    random_seed: int = 20260930
    max_evaluations: int = 150
    optimizer_tolerance: float = 1e-10
    task_mode: str = "auto"

    def __post_init__(self):
        for name in ("orientation_weight", "position_tolerance", "orientation_tolerance",
                     "minimum_limit_margin", "optimizer_tolerance"):
            value = getattr(self, name)
            if isinstance(value, bool) or not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if self.optimizer_tolerance <= np.finfo(float).eps:
            raise ValueError("optimizer_tolerance must exceed machine epsilon")
        if isinstance(self.limit_weight, bool) or not np.isfinite(self.limit_weight) or self.limit_weight < 0:
            raise ValueError("limit_weight must be nonnegative and finite")
        if not 0 < self.minimum_limit_margin <= 0.5:
            raise ValueError("minimum_limit_margin must be in (0, 0.5]")
        for name in ("random_starts", "random_seed", "max_evaluations"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a nonnegative integer")
        if self.max_evaluations == 0:
            raise ValueError("max_evaluations must be positive")
        if self.task_mode not in ("auto", "full", "position"):
            raise ValueError("task_mode must be auto, full or position")


@dataclass(frozen=True)
class IKSolution:
    """一次逆解的结果：关节向量、位置/姿态误差以及候选评分。

    position_error 单位为 m，orientation_error 单位为 rad；cost 只用于
    可行候选之间比较，不是全局最优性的证明。

    Example Input / Output:
        >>> result = IKSolution(np.array([0.2]), 0.0, 0.0, 0.02)
        >>> result.joints.tolist(), result.position_error
        ([0.2], 0.0)
    """
    joints: np.ndarray
    position_error: float
    orientation_error: float
    cost: float


class IKSolver:
    def __init__(self, kinematics: POEKinematics, lower, upper, home,
                 mode: str = "nearest", settings: IKSettings | None = None):
        """配置运动学模型、关节边界、home 初值以及求解模式。

        :param kinematics: POE 正运动学模型。
        :param lower: 每个关节的下限，形状 (dof,)。
        :param upper: 每个关节的上限，必须逐项大于 lower。
        :param home: 多初值模式中的 home 关节坐标。
        :param mode: nearest 使用当前初值；multi-start 搜索多个初值。
        :param settings: IKSettings；None 表示使用默认配置。
        :return: 构造函数不返回值；得到可调用 solve 的求解器。
        :raises ValueError: 模式、边界或输入向量不合法。

        Example Input / Output:
            >>> screws = np.array([[0, 0, 0, 1, 0, 0]]).T
            >>> kinematics = POEKinematics(np.eye(4), screws)
            >>> solver = IKSolver(kinematics, [-1], [1], [0])
            >>> solver.mode, solver.lower.tolist(), solver.upper.tolist()
            ('nearest', [-1.0], [1.0])
        """
        if mode not in IK_MODES:
            raise ValueError(f"mode must be one of {IK_MODES}")
        self.kinematics = kinematics
        self.lower = finite_vector(lower, kinematics.dof, "lower bounds")
        self.upper = finite_vector(upper, kinematics.dof, "upper bounds")
        if np.any(self.lower >= self.upper):
            raise ValueError("lower bounds must be below upper bounds")
        self.home = finite_vector(home, kinematics.dof, "home joints")
        self.mode = mode
        self.settings = settings or IKSettings()
        if not np.isfinite(self.settings.orientation_weight) or self.settings.orientation_weight <= 0:
            raise ValueError("orientation_weight must be positive and finite; use task_mode=position to ignore orientation")
        self.task_space = TaskSpace(kinematics, self.settings.task_mode)
        self.residual_dimension = self.task_space.residual_dimension

    def residual(self, joints: np.ndarray, target: Pose) -> np.ndarray:
        """构造由模型任务子空间决定维数的残差。

        auto 模式只移除模型恒定方向；full 模式保留六维位姿残差。
        此函数只评价某组关节坐标，不进行求解。

        :param joints: 待评价的关节坐标，形状 (dof,)。
        :param target: 目标 Pose。
        :return: 长度 residual_dimension 的数组；位置子空间误差后接加权姿态子空间误差。

        Example Input / Output:
            >>> kinematics = POEKinematics(np.eye(4), np.array([[0, 0, 0, 1, 0, 0]]).T)
            >>> solver = IKSolver(kinematics, [-1], [1], [0])
            >>> solver.residual([0], Pose([0.2, 0, 0], [0, 0, 0, 1])).tolist()
            [-0.2]
        """
        actual = self.kinematics.forward(joints)
        return self.task_space.residual(actual, target, self.settings.orientation_weight)

    def initial_guesses(self, reference: np.ndarray) -> list[np.ndarray]:
        """生成局部优化的初始关节坐标列表。

        nearest 仅以当前坐标启动，名称不保证找到最近解。multi-start
        额外加入 random_starts 组确定性随机初值和边界内的 home 坐标。

        :param reference: 已在 solve 中验证位于关节边界内的参考坐标。
        :return: 若干形状为 (dof,) 的数组组成的列表。

        Example Input / Output:
            >>> kinematics = POEKinematics(np.eye(4), np.array([[0, 0, 0, 1, 0, 0]]).T)
            >>> solver = IKSolver(kinematics, [-1], [1], [0])
            >>> [seed.tolist() for seed in solver.initial_guesses(np.array([0.2]))]
            [[0.2]]
            >>> solver = IKSolver(kinematics, [-1], [1], [0], mode="multi-start")
            >>> len(solver.initial_guesses(np.array([0.2])))
            14
        """
        seeds = [reference]
        if self.mode == "multi-start":
            rng = np.random.default_rng(self.settings.random_seed)
            random_joints = rng.uniform(self.lower, self.upper,
                                        size=(self.settings.random_starts, self.kinematics.dof))
            seeds.extend(random_joints)
            seeds.append(np.clip(self.home, self.lower, self.upper))
        return seeds

    def candidate_cost(self, joints: np.ndarray, reference: np.ndarray) -> float:
        """仅在可行候选间评分：归一化关节位移 + 关节限位惩罚。

        C = sum(((q-q_ref)/span)^2) + w * sum(1/normalized_margin)
        span=q_max-q_min，normalized_margin=min(q-q_min, q_max-q)/span。
        两项都无单位，避免伸缩关节的 m 和旋转关节的 rad 直接比较。
        未做 2*pi 折叠，评分反映执行器实际需要走过的关节角。

        :param joints: 候选关节坐标，形状 (dof,)，应位于边界内。
        :param reference: 当前参考关节坐标，形状 (dof,)。
        :return: 一个非负分数；分数小的可行候选更优先。

        Example Input / Output:
            >>> kinematics = POEKinematics(np.eye(4), np.array([[0, 0, 0, 1, 0, 0]]).T)
            >>> solver = IKSolver(kinematics, [-1], [1], [0])
            >>> round(solver.candidate_cost(np.array([0.0]), np.array([0.0])), 4)
            0.02
            >>> round(solver.candidate_cost(np.array([0.5]), np.array([0.0])), 4)
            0.1025
        """
        travel = (joints - reference) / (self.upper - self.lower)
        margin = np.minimum(joints - self.lower, self.upper - joints) / (self.upper - self.lower)
        margin = np.maximum(margin, self.settings.minimum_limit_margin)
        return float(travel @ travel + self.settings.limit_weight * np.sum(1.0 / margin))

    def is_feasible(self, solution: IKSolution) -> bool:
        """判断候选的位置与姿态误差是否同时达到配置容差。

        :param solution: IKSolution；solve 中的关节边界已由 SciPy 保证。
        :return: 两项误差都不超过容差时为 True，否则为 False。

        Example Input / Output:
            >>> kinematics = POEKinematics(np.eye(4), np.array([[0, 0, 0, 1, 0, 0]]).T)
            >>> solver = IKSolver(kinematics, [-1], [1], [0])
            >>> solver.is_feasible(IKSolution(np.array([0.2]), 0.001, 0.01, 0.02))
            True
            >>> solver.is_feasible(IKSolution(np.array([0.2]), 0.01, 0.01, 0.02))
            False
        """
        return (solution.position_error <= self.settings.position_tolerance
                and (self.settings.task_mode == "position"
                     or solution.orientation_error <= self.settings.orientation_tolerance))

    def candidate_solutions(self, target: Pose, current_joints) -> list[IKSolution]:
        """从一个或多个初值求有界逆解，返回按评分排序的可行候选列表。

        本函数只计算关节目标，不修改 MuJoCo 仿真状态。

        :param target: 与 POE 模型使用相同坐标系的目标 Pose。
        :param current_joints: 当前关节坐标；用作初值和候选评分参考。
        :return: IKSolution 列表，按 cost 从小到大排列，供碰撞筛选及规划使用。
        :raises ValueError: 输入无效，或所有初值都未得到满足容差的解。
            后者不等于证明目标在几何上不可达。

        Example Input / Output:
            >>> kinematics = POEKinematics(np.eye(4), np.array([[0, 0, 0, 1, 0, 0]]).T)
            >>> solver = IKSolver(kinematics, [-1], [1], [0])
            >>> result = solver.candidate_solutions(Pose([0.2, 0, 0], [0, 0, 0, 1]), [0])[0]
            >>> np.round(result.joints, 4).tolist()
            [0.2]
            >>> result.position_error < 1e-6, result.orientation_error < 1e-6
            (True, True)
        """
        reference = finite_vector(current_joints, self.kinematics.dof, "current joints")
        if np.any(reference < self.lower) or np.any(reference > self.upper):
            raise ValueError("current joints are outside limits")
        self.task_space.require_compatible(target, self.settings.position_tolerance,
                                           self.settings.orientation_tolerance)
        if self.residual_dimension == 0:
            position_error, orientation_error = error_norms(self.kinematics.forward(reference), target)
            solution = IKSolution(reference.copy(), position_error, orientation_error,
                                  self.candidate_cost(reference, reference))
            if self.is_feasible(solution):
                return [solution]
            raise ValueError("model has no controllable direction for this target")
        solutions = []
        for seed in self.initial_guesses(reference):
            result = least_squares(
                self.residual, seed, args=(target,), bounds=(self.lower, self.upper),
                x_scale=self.upper - self.lower,
                xtol=self.settings.optimizer_tolerance, ftol=self.settings.optimizer_tolerance,
                gtol=self.settings.optimizer_tolerance, max_nfev=self.settings.max_evaluations,
            )
            position_error, orientation_error = error_norms(self.kinematics.forward(result.x), target)
            solutions.append(IKSolution(result.x.copy(), position_error, orientation_error,
                                        self.candidate_cost(result.x, reference)))

        feasible = [solution for solution in solutions if self.is_feasible(solution)]
        if feasible:
            return sorted(feasible, key=lambda solution: solution.cost)

        # 使用已有结果报告失败，不重复求解；数值失败不能证明目标几何上不可达。
        best = min(solutions, key=lambda solution: solution.position_error**2
                   + (self.settings.orientation_weight * solution.orientation_error)**2)
        raise ValueError(f"IK found no feasible solution: position error {best.position_error:.4f} m, "
                         f"orientation error {best.orientation_error:.4f} rad")


    def solve(self, target: Pose, current_joints) -> IKSolution:
        """保持原来的单解接口，返回评分最小的运动学候选。

        :param target: 目标位姿，位置/m，四元数 xyzw。
        :param current_joints: 当前关节变量；hinge/rad，slide/m。
        :return: IKSolution；该接口只求运动学，实际运动请使用 JointPathPlanner。

        Example Input / Output:
            >>> k = POEKinematics(np.eye(4), np.array([[0, 0, 0, 1, 0, 0]]).T)
            >>> solver = IKSolver(k, [-1], [1], [0])
            >>> np.round(solver.solve(Pose([0.2, 0, 0], [0, 0, 0, 1]), [0]).joints, 3).tolist()
            [0.2]
        """
        return self.candidate_solutions(target, current_joints)[0]
