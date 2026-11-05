"""通用串联机械臂自碰撞工具；仅依赖 NumPy、SciPy 和 POE，不依赖 MuJoCo。

胶囊体的中心线端点在全零关节变量时的空间坐标系中定义，长度单位 m。
joint_count=k 表示该刚体受前 k 个关节影响；0 是固定基座。
模型必须覆盖实际连杆、伸缩组件、夹具和线缆的包络，才能用于实机预检查。
"""

from dataclasses import dataclass
from itertools import combinations
from time import monotonic

import numpy as np

from .kinematics import POEKinematics
from .validation import finite_vector, validate_joint_types


def point_segment_distance(point, start, end) -> float:
    """计算三维点到闭线段的最短距离，允许线段退化为点。

    :param point: 点坐标/m。
    :param start: 线段起点/m。
    :param end: 线段终点/m。
    :return: 最短距离/m。

    Example Input / Output:
        >>> point_segment_distance([0.5, 1, 0], [0, 0, 0], [1, 0, 0])
        1.0
    """
    point = finite_vector(point, 3, "point")
    start = finite_vector(start, 3, "segment start")
    end = finite_vector(end, 3, "segment end")
    direction = end - start
    squared_length = float(direction @ direction)
    fraction = np.clip((point - start) @ direction / squared_length, 0, 1) if squared_length else 0.0
    return float(np.linalg.norm(point - (start + fraction * direction)))


def segment_distance(first_start, first_end, second_start, second_end) -> float:
    """计算两条三维闭线段的最短距离，兼容平行线、交叉线和退化线段。

    先检查四个端点到另一线段的距离，再检查无限直线的内部最近点。
    近乎平行时使用最小二乘，避免直接除以很小的行列式。

    :param first_start: 第一线段起点/m。
    :param first_end: 第一线段终点/m。
    :param second_start: 第二线段起点/m。
    :param second_end: 第二线段终点/m。
    :return: 中心线的最短距离/m；胶囊体表面间距还要减去两半径。

    Example Input / Output:
        >>> segment_distance([0, 0, 0], [1, 0, 0], [0.5, -1, 0], [0.5, 1, 0])
        0.0
        >>> segment_distance([0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0])
        1.0
    """
    a = finite_vector(first_start, 3, "first start")
    b = finite_vector(first_end, 3, "first end")
    c = finite_vector(second_start, 3, "second start")
    d = finite_vector(second_end, 3, "second end")
    distances = [point_segment_distance(a, c, d), point_segment_distance(b, c, d),
                 point_segment_distance(c, a, b), point_segment_distance(d, a, b)]
    u, v, w = b - a, d - c, a - c
    uu, vv, uv = float(u @ u), float(v @ v), float(u @ v)
    determinant = uu * vv - uv * uv
    if uu > 0 and vv > 0:
        if determinant > 1e-8 * uu * vv:
            uw, vw = float(u @ w), float(v @ w)
            s = (uv * vw - vv * uw) / determinant
            t = (uu * vw - uv * uw) / determinant
        else:
            s, t = np.linalg.lstsq(np.column_stack((u, -v)), c - a, rcond=None)[0]
        if 0 <= s <= 1 and 0 <= t <= 1:
            distances.append(float(np.linalg.norm(a + s * u - c - t * v)))
    return min(distances)


@dataclass(frozen=True)
class Capsule:
    """用零位中心线和半径定义一个刚体包络；同一连杆可使用多个胶囊体。"""

    name: str
    joint_count: int
    start: np.ndarray
    end: np.ndarray
    radius: float

    def __post_init__(self):
        """检查并复制胶囊体参数，端点保存为只读数组。

        :return: None；radius/m 必须大于零，joint_count 必须是非负整数。

        """
        if not isinstance(self.name, str) or not self.name:
            raise ValueError("capsule name must be nonempty")
        if isinstance(self.joint_count, bool) or not isinstance(self.joint_count, (int, np.integer)) or self.joint_count < 0:
            raise ValueError("joint_count must be a nonnegative integer")
        radius = float(finite_vector([self.radius], 1, "radius")[0])
        if radius <= 0:
            raise ValueError("capsule radius must be positive")
        for attribute in ("start", "end"):
            value = finite_vector(getattr(self, attribute), 3, attribute)
            value.setflags(write=False)
            object.__setattr__(self, attribute, value)
        object.__setattr__(self, "radius", radius)


@dataclass(frozen=True)
class SelfContact:
    """两胶囊体的间距结果；clearance/m 小于零表示包络相互穿透。

    Example Input / Output:
        >>> contact = SelfContact("base", "tool", -0.01)
        >>> contact.clearance
        -0.01
    """

    first: str
    second: str
    clearance: float
    required_clearance: float = 0.003


class SelfCollisionModel:
    def __init__(self, kinematics: POEKinematics, joint_types, lower, upper, capsules,
                 allowed_pairs=(), safety_margin: float = 0.003, max_edge_depth: int = 14, joint_uncertainty=None):
        """创建与仿真及具体驱动无关的自碰撞模型。

        :param kinematics: POE 模型，各旋转轴和伸缩方向必须归一化。
        :param joint_types: 任意数量的 hinge/slide，按运动链排列。
        :param lower: 每关节下限；hinge/rad，slide/m。
        :param upper: 每关节上限，同 lower；要求有限且上限大于下限。
        :param capsules: 完整机械臂的胶囊体列表，端点为零位空间坐标/m。
        :param allowed_pairs: 明确允许重叠的胶囊体名称对；默认不忽略相邻连杆。
            同一刚体上的胶囊体不互检，因为它们没有相对运动。
        :param safety_margin: 基础表面间距要求/m；覆盖几何建模和标定误差。
            joint_uncertainty 换算出的运动误差包络另行叠加。
        :param joint_uncertainty: 各关节最大位置/跟踪不确定度，hinge/rad，slide/m。
            None 表示只检查理想运动学；执行监测器要求配置正的不确定度。
        :param max_edge_depth: 自适应路径检查的最大细分深度；无法确认时拒绝。
        :return: 构造函数无返回值；保存模型副本，不修改传入的运动学对象。

        """
        self.joint_types = validate_joint_types(joint_types)
        self.dof = len(self.joint_types)
        if self.dof != kinematics.dof:
            raise ValueError("joint types must match kinematics DOF")
        self.kinematics = POEKinematics(kinematics.zero_transform, kinematics.screw_axes)
        self.lower = finite_vector(lower, self.dof, "lower")
        self.upper = finite_vector(upper, self.dof, "upper")
        if np.any(self.lower >= self.upper):
            raise ValueError("lower must be below upper")
        self.safety_margin = float(finite_vector([safety_margin], 1, "safety margin")[0])
        if self.safety_margin < 0:
            raise ValueError("safety margin must be nonnegative")
        if isinstance(max_edge_depth, bool) or not isinstance(max_edge_depth, int) or not 1 <= max_edge_depth <= 24:
            raise ValueError("max_edge_depth must be an integer in [1, 24]")
        self.max_edge_depth = max_edge_depth
        self.joint_uncertainty = (np.zeros(self.dof) if joint_uncertainty is None
                                  else finite_vector(joint_uncertainty, self.dof, "joint uncertainty"))
        if np.any(self.joint_uncertainty < 0):
            raise ValueError("joint uncertainty must be nonnegative")
        self.capsules = tuple(capsules)
        if not self.capsules or any(not isinstance(c, Capsule) for c in self.capsules):
            raise ValueError("provide a nonempty sequence of Capsule objects")
        names = {c.name for c in self.capsules}
        if len(names) != len(self.capsules) or any(c.joint_count > self.dof for c in self.capsules):
            raise ValueError("capsule names must be unique and joint_count must not exceed DOF")
        allowed = set()
        for pair in allowed_pairs:
            if isinstance(pair, str) or len(pair) != 2 or pair[0] == pair[1] or not set(pair) <= names:
                raise ValueError("allowed_pairs must contain two distinct existing capsule names")
            allowed.add(frozenset(pair))
        self.allowed_pairs = tuple(tuple(sorted(pair)) for pair in sorted(allowed, key=lambda p: sorted(p)))
        self.pairs = tuple((i, j) for i, j in combinations(range(len(self.capsules)), 2)
                           if self.capsules[i].joint_count != self.capsules[j].joint_count
                           and frozenset((self.capsules[i].name, self.capsules[j].name)) not in allowed)
        self.axes = self.kinematics.screw_axes[:3].T.copy()
        self.anchors = np.zeros((self.dof, 3))
        self.angular_rates = np.zeros(self.dof)
        self.linear_rates = np.zeros(self.dof)
        for i, kind in enumerate(self.joint_types):
            omega = self.kinematics.screw_axes[:3, i]
            velocity = self.kinematics.screw_axes[3:, i]
            if kind == "hinge":
                if not np.isclose(np.linalg.norm(omega), 1, atol=1e-8, rtol=0) or abs(omega @ velocity) > 1e-8:
                    raise ValueError("hinge screw must have unit omega and zero pitch")
                self.angular_rates[i] = np.linalg.norm(omega)
                self.anchors[i] = np.cross(omega, velocity) / float(omega @ omega)
                self.linear_rates[i] = abs(float(omega @ velocity)) / self.angular_rates[i]
            elif np.any(omega != 0) or not np.isclose(np.linalg.norm(velocity), 1, atol=1e-8, rtol=0):
                raise ValueError("slide screw must have zero omega and unit translation")
            else:
                self.linear_rates[i] = np.linalg.norm(velocity)
        for value in (self.lower, self.upper, self.kinematics.screw_axes, self.kinematics.zero_transform, self.joint_uncertainty):
            value.setflags(write=False)
        self.hinge_levers = self._lever_bounds()
        self.uncertainty_padding = self._uncertainty_padding()
        for value in (self.axes, self.anchors, self.angular_rates, self.linear_rates,
                      self.hinge_levers, self.uncertainty_padding):
            value.setflags(write=False)

    def _lever_bounds(self) -> np.ndarray:
        """计算旋转关节到胶囊体中心线的保守力臂上界/m。

        使用零位轴上点之间的距离和末端中心线到最后轴上点的距离。
        路径中的下游伸缩量另在 displacement_bounds 中加入。

        :return: 胶囊体数量 x DOF 的非负矩阵。

        """
        bounds = np.zeros((len(self.capsules), self.dof))
        for row, capsule in enumerate(self.capsules):
            k = capsule.joint_count
            if k == 0:
                continue
            tip_distance = max(np.linalg.norm(capsule.start - self.anchors[k - 1]),
                               np.linalg.norm(capsule.end - self.anchors[k - 1]))
            for joint in range(k):
                lengths = np.linalg.norm(np.diff(self.anchors[joint:k], axis=0), axis=1)
                bounds[row, joint] = tip_distance + float(np.sum(lengths))
        return bounds

    def _uncertainty_padding(self) -> np.ndarray:
        """把各关节不确定度转换成每个胶囊体的空间运动上界/m。

        取整个关节限位窗口及误差范围内的最坏力臂，额外扩大自碰撞间距。
        不能自动推断这些误差，实机应根据标定和反馈验证配置。

        :return: 每个胶囊体的保守空间包络增量/m。

        """
        max_extension = np.maximum(np.abs(self.lower), np.abs(self.upper)) + self.joint_uncertainty
        padding = np.zeros(len(self.capsules))
        for row, capsule in enumerate(self.capsules):
            for joint in range(capsule.joint_count):
                downstream = np.sum(max_extension[joint:capsule.joint_count]
                                    * self.linear_rates[joint:capsule.joint_count])
                rate = self.angular_rates[joint] * (self.hinge_levers[row, joint] + downstream) + self.linear_rates[joint]
                padding[row] += rate * self.joint_uncertainty[joint]
        return padding

    def validate_joint_variables(self, joint_variables) -> np.ndarray:
        """复制并检查反馈/目标关节变量，不裁剪、不折叠旋转角。

        :param joint_variables: 一维关节变量；hinge/rad，slide/m。
        :return: 有限且在上下限内的数组副本。
        :raises ValueError: 维数不符、非有限或超出关节限位。

        """
        value = finite_vector(joint_variables, self.dof, "joint variables")
        if np.any(value < self.lower) or np.any(value > self.upper):
            raise ValueError("joint variables are outside limits")
        return value

    def transformed_capsules(self, joint_variables) -> tuple[Capsule, ...]:
        """按 POE 前缀变换计算每个连杆胶囊体的当前位置。

        第 k 刚体使用 exp([S1]q1)...exp([Sk]qk) 变换零位端点。
        基座的 k=0 使用单位变换；不需要重新读取仿真模型。

        :param joint_variables: 当前关节变量；hinge/rad，slide/m。
        :return: 当前空间坐标下的胶囊体元组，端点/m。

        """
        value = self.validate_joint_variables(joint_variables)
        transforms = self.kinematics.prefix_transforms(value)
        result = []
        for capsule in self.capsules:
            transform = transforms[capsule.joint_count]
            start = transform[:3, :3] @ capsule.start + transform[:3, 3]
            end = transform[:3, :3] @ capsule.end + transform[:3, 3]
            result.append(Capsule(capsule.name, capsule.joint_count, start, end, capsule.radius))
        return tuple(result)

    def clearances(self, joint_variables) -> tuple[SelfContact, ...]:
        """计算所有需检查的自身几何对的表面间距，包括尚未碰撞的几何对。

        :param joint_variables: 关节变量；hinge/rad，slide/m。
        :return: SelfContact 元组；每项 clearance/m 为中心线距离减两半径。

        """
        current = self.transformed_capsules(joint_variables)
        result = []
        for i, j in self.pairs:
            first, second = current[i], current[j]
            clearance = segment_distance(first.start, first.end, second.start, second.end) - first.radius - second.radius
            result.append(SelfContact(first.name, second.name, clearance,
                                      self.safety_margin + self.uncertainty_padding[i] + self.uncertainty_padding[j]))
        return tuple(result)

    def contacts(self, joint_variables) -> tuple[SelfContact, ...]:
        """返回已碰撞或未满足安全间距的几何对。

        :param joint_variables: 关节变量；hinge/rad，slide/m。
        :return: SelfContact 元组；为空表示当前模型满足间距要求。

        """
        return tuple(contact for contact in self.clearances(joint_variables) if contact.clearance <= contact.required_clearance)

    def require_valid(self, joint_variables) -> None:
        """检查关节限位和自身间距；失败时报告具体几何对。

        :param joint_variables: 反馈或目标关节变量；hinge/rad，slide/m。
        :return: None 表示当前构型通过。
        :raises ValueError: 输入无效、超限、碰撞或安全间距不足。

        """
        contacts = self.contacts(joint_variables)
        if contacts:
            first = contacts[0]
            raise ValueError(f"self collision / insufficient clearance: {first.first} / {first.second}; "
                             f"clearance={first.clearance:.6f} m, required>{first.required_clearance:.6f} m")

    def is_valid(self, joint_variables) -> bool:
        """以布尔值检查关节限位和自身碰撞，供规划器筛选采样点。

        :param joint_variables: 关节变量；hinge/rad，slide/m。
        :return: 输入、限位和自身间距都有效时为 True。

        """
        try:
            self.require_valid(joint_variables)
        except ValueError:
            return False
        return True

    def displacement_bounds(self, start, goal) -> np.ndarray:
        """估计一段关节直线路径上每个胶囊体中心线的最大移动距离/m。

        对 hinge 累加 力臂上界*角变化，对 slide 累加伸缩变化；力臂中
        包含全部下游 slide 在该段上的最大绝对伸缩量。返回值用于自适应
        路径检查，不是实际路径长度，也不直接比较 rad 和 m。

        :param start: 段起点关节变量。
        :param goal: 段终点关节变量。
        :return: 每个胶囊体的运动距离上界/m。

        """
        start = self.validate_joint_variables(start)
        goal = self.validate_joint_variables(goal)
        delta = np.abs(goal - start)
        max_extension = np.maximum(np.abs(start), np.abs(goal))
        distances = np.zeros(len(self.capsules))
        for row, capsule in enumerate(self.capsules):
            for joint in range(capsule.joint_count):
                if self.joint_types[joint] == "slide":
                    distances[row] += self.linear_rates[joint] * delta[joint]
                else:
                    downstream = sum(max_extension[j] * self.linear_rates[j]
                                     for j in range(joint, capsule.joint_count))
                    distances[row] += ((self.hinge_levers[row, joint] + downstream) * self.angular_rates[joint]
                                       + self.linear_rates[joint]) * delta[joint]
        return distances

    def require_edge(self, start, goal, deadline: float | None = None) -> None:
        """自适应检查整段关节直线路径，不能确认的区间也会拒绝。

        计算区间中点的表面间距与该区间内每个几何体的最大运动上界。
        若所有几何对的间距大于安全间距+双方运动上界，则整段通过；
        否则继续二分。这里使用解析运动上界，避免仅依赖固定稀疏采样。
        保证范围是所给胶囊体模型与理想关节插值，受浮点精度约束。

        :param start: 起点关节变量；hinge/rad，slide/m。
        :param goal: 终点关节变量。
        :param deadline: 可选 monotonic 绝对截止时间/s，供规划器控制搜索预算。
        :return: None 表示每个路径区间均通过保守间距检查。
        :raises ValueError: 起终点无效、途中碰撞或细分深度内无法确认。
        :raises TimeoutError: 超过截止时间。

        """
        start = self.validate_joint_variables(start)
        goal = self.validate_joint_variables(goal)
        self.require_valid(start)
        self.require_valid(goal)
        pending = [(start, goal, 0)]
        while pending:
            if deadline is not None and monotonic() >= deadline:
                raise TimeoutError("self-collision path check exceeded deadline")
            first, last, depth = pending.pop()
            middle = (first + last) / 2
            contacts = self.clearances(middle)
            if any(c.clearance <= c.required_clearance for c in contacts):
                self.require_valid(middle)
            bounds = self.displacement_bounds(first, last) / 2
            certified = all(c.clearance > c.required_clearance + bounds[i] + bounds[j] + 1e-10
                            for c, (i, j) in zip(contacts, self.pairs))
            if certified:
                continue
            if depth >= self.max_edge_depth:
                raise ValueError("cannot certify self-collision clearance within maximum edge subdivision depth")
            pending.append((middle, last, depth + 1))
            pending.append((first, middle, depth + 1))

    def edge_is_valid(self, start, goal, deadline: float | None = None) -> bool:
        """以布尔值检查整个关节直线段；超时仍向调用者抛出异常。

        :param start: 起点关节变量。
        :param goal: 终点关节变量。
        :param deadline: 可选搜索截止时间/s。
        :return: 整段可确认满足间距要求时为 True。

        """
        try:
            self.require_edge(start, goal, deadline)
        except ValueError:
            return False
        return True
