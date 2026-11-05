"""不依赖仿真或硬件 SDK 的关节空间 RRT-Connect 自碰撞规避规划。

参考 Kuffner/LaValle 的双树 EXTEND/CONNECT 思路。搜索距离按每个关节
范围归一化；每条树边和每次路径缩短都调用同一保守自碰撞检查器。
"""

from dataclasses import dataclass
from time import monotonic

import numpy as np

from .collision import SelfCollisionModel
from .validation import finite_vector


class PlanningError(RuntimeError):
    """搜索预算内未找到路径；这不等于证明目标不可达。"""


@dataclass(frozen=True)
class PlannerSettings:
    """路径规划参数；step_size 是归一化关节空间中的无单位步长。

    Example Input / Output:
        >>> settings = PlannerSettings()
        >>> settings.step_size, settings.time_limit
        (0.12, 10.0)
    """

    step_size: float = 0.12
    time_limit: float = 10.0
    max_iterations: int = 3000
    goal_bias: float = 0.15
    shortcut_attempts: int = 40
    random_seed: int = 20261002


@dataclass
class _Tree:
    """RRT 内部树；nodes 存归一化关节变量，parents 存父节点索引。"""

    nodes: list[np.ndarray]
    parents: list[int]


class JointPathPlanner:
    def __init__(self, model: SelfCollisionModel, settings: PlannerSettings | None = None):
        """绑定几何模型和搜索参数，不连接驱动，不修改机械臂状态。

        :param model: 自碰撞模型，支持任意数量 hinge/slide。
        :param settings: PlannerSettings；None 使用默认参数。
        :return: 构造函数无返回值；得到可调用 plan/plan_pose 的规划器。

        """
        self.model = model
        self.settings = settings or PlannerSettings()
        numbers = finite_vector([self.settings.step_size, self.settings.time_limit,
                                 self.settings.goal_bias], 3, "planner settings")
        if numbers[0] <= 0 or numbers[1] <= 0 or not 0 <= numbers[2] <= 1:
            raise ValueError("step/time must be positive; goal bias must be in [0,1]")
        for name in ("max_iterations", "shortcut_attempts", "random_seed"):
            value = getattr(self.settings, name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"{name} must be a nonnegative integer")
        if self.settings.max_iterations == 0:
            raise ValueError("max_iterations must be positive")
        self.span = model.upper - model.lower

    def _joint_variables(self, normalized) -> np.ndarray:
        """将 [0,1] 中的归一化变量还原成各关节自己的单位。

        :param normalized: 一维归一化关节数组。
        :return: 关节变量；hinge/rad，slide/m。

        """
        return self.model.lower + np.clip(normalized, 0, 1) * self.span

    def _grow(self, tree: _Tree, target, deadline: float) -> tuple[int | None, bool]:
        """将一棵树向目标延伸最多一个步长，树边必须通过路径检查。

        :param tree: 当前树，节点使用归一化变量。
        :param target: 归一化目标节点。
        :param deadline: 搜索截止时间，monotonic/s。
        :return: (新节点索引或 None, 是否到达目标)；None 表示被碰撞阻挡。

        """
        if monotonic() >= deadline:
            raise TimeoutError("planning time limit reached")
        distances = np.linalg.norm(np.asarray(tree.nodes) - target, axis=1)
        nearest = int(np.argmin(distances))
        distance = float(distances[nearest])
        if distance < 1e-12:
            return nearest, True
        fraction = min(1.0, self.settings.step_size / distance)
        new_node = tree.nodes[nearest] + fraction * (target - tree.nodes[nearest])
        if not self.model.edge_is_valid(self._joint_variables(tree.nodes[nearest]),
                                        self._joint_variables(new_node), deadline):
            return None, False
        tree.nodes.append(new_node)
        tree.parents.append(nearest)
        return len(tree.nodes) - 1, fraction == 1.0

    def _trace(self, tree: _Tree, index: int) -> list[np.ndarray]:
        """沿父索引从某节点回溯到根节点，再按根到节点顺序返回。

        :param tree: 保存 nodes/parents 的树。
        :param index: 树中节点索引。
        :return: 归一化节点数组的副本列表。

        """
        result = []
        while index >= 0:
            result.append(tree.nodes[index].copy())
            index = tree.parents[index]
        return list(reversed(result))

    def _shortcut(self, path: np.ndarray, rng, deadline: float) -> np.ndarray:
        """尝试用已验证的关节直线替换部分绕行节点，不改变起终点。

        :param path: 已通过路径检查的关节路点数组。
        :param rng: NumPy 随机数生成器。
        :param deadline: 截止时间/s；预算用完就返回已有有效路径。
        :return: 节点数量不增加的路径副本。

        """
        result = path.copy()
        for _ in range(self.settings.shortcut_attempts):
            if len(result) < 3 or monotonic() >= deadline:
                break
            first, last = sorted(rng.choice(len(result), 2, replace=False).tolist())
            if last <= first + 1:
                continue
            try:
                clear = self.model.edge_is_valid(result[first], result[last], deadline)
            except TimeoutError:
                break
            if clear:
                result = np.vstack((result[:first + 1], result[last:]))
        return result

    def plan(self, current_joint_variables, target_joint_variables,
             time_limit: float | None = None) -> np.ndarray:
        """从实际反馈关节变量规划到目标，直线路径不通时用双树搜索绕行。

        :param current_joint_variables: 当前反馈关节变量；hinge/rad，slide/m。
        :param target_joint_variables: 目标关节变量，同当前数组维数和单位。
        :param time_limit: 可选本次搜索时间预算/s；默认取 settings.time_limit。
        :return: (路点数, DOF) 的数组，包括起点和终点；每段均通过自碰撞检查。
        :raises ValueError: 起终点无效、超限或已经发生自碰撞。
        :raises PlanningError: 搜索预算内未找到可确认的路径。

        """
        start = self.model.validate_joint_variables(current_joint_variables)
        goal = self.model.validate_joint_variables(target_joint_variables)
        self.model.require_valid(start)
        self.model.require_valid(goal)
        budget = self.settings.time_limit if time_limit is None else float(time_limit)
        if not np.isfinite(budget) or budget <= 0:
            raise ValueError("planning time limit must be positive and finite")
        deadline = monotonic() + budget
        rng = np.random.default_rng(self.settings.random_seed)
        if np.array_equal(start, goal):
            return start[None, :].copy()
        try:
            if self.model.edge_is_valid(start, goal, deadline):
                return np.vstack((start, goal))
            first_tree = _Tree([(start - self.model.lower) / self.span], [-1])
            second_tree = _Tree([(goal - self.model.lower) / self.span], [-1])
            first_is_start = True
            for _ in range(self.settings.max_iterations):
                if monotonic() >= deadline:
                    break
                target = (second_tree.nodes[0] if rng.random() < self.settings.goal_bias
                          else rng.uniform(0, 1, self.model.dof))
                first_index, _ = self._grow(first_tree, target, deadline)
                if first_index is not None:
                    meeting = first_tree.nodes[first_index]
                    while True:
                        second_index, reached = self._grow(second_tree, meeting, deadline)
                        if second_index is None:
                            break
                        if reached:
                            first_path = self._trace(first_tree, first_index)
                            second_path = self._trace(second_tree, second_index)
                            combined = (first_path + second_path[-2::-1] if first_is_start
                                        else second_path + first_path[-2::-1])
                            path = np.array([self._joint_variables(p) for p in combined])
                            # 精确保留调用者起终点，归一化造成的浮点误差不传给驱动。
                            path[0], path[-1] = start, goal
                            return self._shortcut(path, rng, deadline)
                first_tree, second_tree = second_tree, first_tree
                first_is_start = not first_is_start
        except TimeoutError:
            pass
        raise PlanningError("no certified self-collision-free path found within search budget; "
                            "this does not prove the target unreachable")

    def plan_pose(self, current_joint_variables, target_pose, ik_settings=None) -> np.ndarray:
        """对目标位姿生成多个逆解，并寻找可执行的自身避碰关节路径。

        优先尝试所有有效逆解的直线路径，再在剩余预算内对各分支运行 RRT。
        用反馈关节变量作为起点；不会因为末端位姿误差小就接受碰撞构型。

        :param current_joint_variables: 当前反馈关节变量；hinge/rad，slide/m。
        :param target_pose: Pose，位置/m、xyzw 四元数/无单位。
        :param ik_settings: 可选 IKSettings，用于逆解容差和初值数量。
        :return: 自身避碰关节路点数组；末端最后一项达到目标位姿容差。
        :raises ValueError: 输入无效或所有逆解目标构型有碰撞。
        :raises PlanningError: 未找到满足运动学/路径要求的方案。

        """
        from .inverse_kinematics import IKSolver

        start = self.model.validate_joint_variables(current_joint_variables)
        self.model.require_valid(start)
        solver = IKSolver(self.model.kinematics, self.model.lower, self.model.upper,
                          start, mode="multi-start", settings=ik_settings)
        try:
            solutions = solver.candidate_solutions(target_pose, start)
        except ValueError as exc:
            raise PlanningError(f"pose IK failed: {exc}") from exc
        valid = [solution for solution in solutions if self.model.is_valid(solution.joints)]
        if not valid:
            raise ValueError("all pose IK candidates violate self-collision clearance")
        deadline = monotonic() + self.settings.time_limit
        try:
            for solution in valid:
                if self.model.edge_is_valid(start, solution.joints, deadline):
                    return np.vstack((start, solution.joints))
        except TimeoutError as exc:
            raise PlanningError("pose path validation exceeded search budget") from exc
        for index, solution in enumerate(valid):
            remaining = deadline - monotonic()
            if remaining <= 0:
                break
            try:
                return self.plan(start, solution.joints, time_limit=remaining / (len(valid) - index))
            except PlanningError:
                continue
        raise PlanningError("no self-collision-free pose motion found within search budget")
