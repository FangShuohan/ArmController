"""唯一建模入口：每个电机关节的参数、参数检查和 deg↔SI 映射。"""
from dataclasses import asdict, dataclass, field
import json
from pathlib import Path

import numpy as np

from ArmController import Capsule, POEKinematics, Pose, SelfCollisionModel
from ArmController.validation import finite_matrix, finite_vector, unit_vectors


def _number(value, name, *, positive=False):
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a number")
    value = float(finite_vector([value], 1, name)[0])
    if positive and value <= 0:
        raise ValueError(f"{name} must be positive")
    return value


@dataclass(frozen=True)
class MotorJoint:
    """按基座→工具顺序填写逻辑关节；默认独立传动，可选 motor_coupling。

    axis_xyz / axis_point_m：全零位同一空间坐标系下的单位轴向/轴上一点。
    hinge 的 lower/upper/uncertainty 为 deg；slide 为 m。
    gear_ratio：每输出轴一转所需的电机转数；UART 位置是该电机轴的 deg。
    若协议已经是减速后角度，则 gear_ratio=1，避免重复换算。
    slide 另填 screw_lead_m（输出丝杠一转的行程），其余同样换成电机 deg。
    耦合时轴线/限位/gear_ratio/direction 按关节列，ID/zero/tolerance 按电机行。
    tolerance_deg 是电机侧跟踪容差，必须被 uncertainty 覆盖。
    """
    motor_id: int
    axis_xyz: tuple
    axis_point_m: tuple
    lower: float
    upper: float
    joint_type: str = "hinge"
    gear_ratio: float = 1.0
    direction: int = 1
    zero_deg: float = 0.0
    screw_lead_m: float | None = None
    uncertainty: float = 0.5
    tolerance_deg: float = 0.3
    stopped_deg_s: float = 0.1
    name: str = ""  # 可选逻辑关节名称，仅用于建模/界面说明

    def __post_init__(self):
        if not isinstance(self.name, str):
            raise ValueError("joint name must be a string")
        if isinstance(self.motor_id, bool) or not isinstance(self.motor_id, int) or not 1 <= self.motor_id <= 255:
            raise ValueError("motor_id must be an integer in [1,255]")
        if self.joint_type not in ("hinge", "slide"):
            raise ValueError("joint_type must be hinge or slide")
        if isinstance(self.direction, bool) or not isinstance(self.direction, int) or self.direction not in (-1, 1):
            raise ValueError("direction must be +1 or -1")
        axis = unit_vectors([self.axis_xyz], 1, "axis_xyz")[0]
        anchor = finite_vector(self.axis_point_m, 3, "axis_point_m")
        object.__setattr__(self, "axis_xyz", tuple(axis))
        object.__setattr__(self, "axis_point_m", tuple(anchor))
        for name in ("lower", "upper", "zero_deg", "uncertainty"):
            object.__setattr__(self, name, _number(getattr(self, name), name))
        for name in ("gear_ratio", "tolerance_deg", "stopped_deg_s"):
            object.__setattr__(self, name, _number(getattr(self, name), name, positive=True))
        if self.lower >= self.upper:
            raise ValueError("lower must be below upper")
        if not 0 < self.uncertainty < (self.upper - self.lower) / 2:
            raise ValueError("uncertainty must be positive and smaller than half the joint range")
        if self.joint_type == "slide":
            object.__setattr__(self, "screw_lead_m", _number(self.screw_lead_m, "screw_lead_m", positive=True))
        elif self.screw_lead_m is not None:
            raise ValueError("screw_lead_m is only valid for slide joints")
        if not np.isfinite(self.motor_scale) or self.motor_scale == 0:
            raise ValueError("motor conversion scale must be finite and nonzero")
        if self.tolerance_deg > abs(self.motor_scale) * self.uncertainty_si:
            raise ValueError("motor tolerance_deg must be covered by joint uncertainty")

    @property
    def motor_scale(self):
        """电机 deg / 关节 SI 单位（hinge rad，slide m）。"""
        factor = 180 / np.pi if self.joint_type == "hinge" else 360 / self.screw_lead_m
        return self.direction * self.gear_ratio * factor

    @property
    def uncertainty_si(self):
        return np.deg2rad(self.uncertainty) if self.joint_type == "hinge" else self.uncertainty


@dataclass(frozen=True)
class ArmModel:
    joints: tuple[MotorJoint, ...]
    collision: SelfCollisionModel
    motor_coupling: np.ndarray | None = field(default=None, repr=False, compare=False)
    motor_bounds_deg: np.ndarray | None = field(default=None, repr=False, compare=False)
    _inverse_coupling: np.ndarray = field(init=False, repr=False, compare=False)

    def __post_init__(self):
        # 列：逻辑关节经原 scale 换算后的 deg；行：motor_ids 对应物理电机。
        coupling = (np.eye(self.dof) if self.motor_coupling is None else
                    finite_matrix(self.motor_coupling, (self.dof, self.dof), "motor_coupling"))
        condition = np.linalg.cond(coupling)
        if not np.isfinite(condition) or condition > 1e10:
            raise ValueError("motor_coupling must be invertible and well-conditioned")
        inverse = np.linalg.inv(coupling)
        if not np.all(np.isfinite(inverse)):
            raise ValueError("inverse motor_coupling must be finite")
        for name, value in (("motor_coupling", coupling), ("_inverse_coupling", inverse)):
            value.setflags(write=False)
            object.__setattr__(self, name, value)
        if self.motor_bounds_deg is not None:
            bounds = finite_matrix(self.motor_bounds_deg, (self.dof, 2), "motor_bounds_deg")
            if np.any(bounds[:, 0] >= bounds[:, 1]):
                raise ValueError("motor lower bounds must be below upper bounds")
            bounds.setflags(write=False)
            object.__setattr__(self, "motor_bounds_deg", bounds)
        if np.any(self.joint_tolerance > np.array([j.uncertainty_si for j in self.joints]) * (1 + 1e-12)):
            raise ValueError("coupled motor tolerances must be covered by joint uncertainty")

    @property
    def joint_tolerance(self):
        """物理电机误差经逆传动映射得到的关节误差上界，单位 rad/m。"""
        return (np.abs(self._inverse_coupling) @ self.tolerance_deg) / np.abs(self.scale)

    def _validate_motor(self, motor_deg):
        motor = finite_vector(motor_deg, self.dof, "motor_deg")
        if self.motor_bounds_deg is not None and (np.any(motor < self.motor_bounds_deg[:, 0])
                                                  or np.any(motor > self.motor_bounds_deg[:, 1])):
            raise ValueError("motor angles are outside configured motor bounds")
        return motor

    @property
    def motor_ids(self):
        return tuple(j.motor_id for j in self.joints)

    @property
    def dof(self):
        return len(self.joints)

    @property
    def kinematics(self):
        return self.collision.kinematics

    @property
    def scale(self):
        return np.array([j.motor_scale for j in self.joints])

    @property
    def zero_deg(self):
        return np.array([j.zero_deg for j in self.joints])

    @property
    def tolerance_deg(self):
        return np.array([j.tolerance_deg for j in self.joints])

    @property
    def stopped_deg_s(self):
        return np.array([j.stopped_deg_s for j in self.joints])

    def motor_to_joints(self, motor_deg):
        """绝对电机 deg → 内部关节 rad/m；同时检查关节限位。"""
        q = (self._inverse_coupling @ (self._validate_motor(motor_deg) - self.zero_deg)) / self.scale
        return self.collision.validate_joint_variables(q)

    def joints_to_motor(self, q):
        """内部关节 rad/m → 绝对电机 deg；不折回到单圈。"""
        return self._validate_motor(self.zero_deg + self.motor_coupling @
                                    (self.scale * self.collision.validate_joint_variables(q)))

    def user_to_joints(self, values):
        """用户关节 deg/m → 内部 rad/m。"""
        q = finite_vector(values, self.dof, "joint positions (deg/m)")
        for i, joint in enumerate(self.joints):
            if joint.joint_type == "hinge":
                q[i] = np.deg2rad(q[i])
        return self.collision.validate_joint_variables(q)


def build_model(joints, *, tool_position_m, tool_quaternion_xyzw=(0, 0, 0, 1),
                capsules, allowed_pairs=(), safety_margin_m=0.003,
                motor_coupling=None, motor_bounds_deg=None):
    """用户传参入口。运动学轴线、工具和胶囊体均来自全零位空间坐标。

    motor_coupling 是无单位可逆矩阵，motor=zero+C@(scale*q)，默认 C=I。
    motor_bounds_deg 为物理电机角度限位 (N,2)，独立于逻辑关节限位。
    capsules 是显式的真实外形包络，不自动猜测杆长或排除相邻部件。
    一般参数检查只能证明数值/结构自洽，不能替代 CAD/实机标定。
    """
    joints = tuple(joints)
    if not joints or any(not isinstance(j, MotorJoint) for j in joints):
        raise ValueError("joints must be a nonempty sequence of MotorJoint")
    if len({j.motor_id for j in joints}) != len(joints):
        raise ValueError("motor IDs must be unique")
    types = [j.joint_type for j in joints]
    kinematics = POEKinematics.from_joints(
        [j.axis_xyz for j in joints], [j.axis_point_m for j in joints], types,
        Pose(tool_position_m, tool_quaternion_xyzw),
    )
    lower = [np.deg2rad(j.lower) if j.joint_type == "hinge" else j.lower for j in joints]
    upper = [np.deg2rad(j.upper) if j.joint_type == "hinge" else j.upper for j in joints]
    collision = SelfCollisionModel(
        kinematics, types, lower, upper, capsules, allowed_pairs=allowed_pairs,
        safety_margin=safety_margin_m, joint_uncertainty=[j.uncertainty_si for j in joints],
    )
    for joint, low, high in zip(joints, lower, upper):
        if not np.all(np.isfinite([joint.zero_deg + joint.motor_scale * low,
                                  joint.zero_deg + joint.motor_scale * high])):
            raise ValueError("motor positions implied by limits must be finite")
    return ArmModel(joints, collision, motor_coupling, motor_bounds_deg)


def model_from_dict(data):
    """从通用 JSON 对象建模。也接受含 model/uart/control 的配置，只读取其中 model。"""
    if not isinstance(data, dict):
        raise ValueError("模型 JSON 必须是对象")
    raw = data.get("model", data)
    if not isinstance(raw, dict):
        raise ValueError("model 必须是对象")
    values = dict(raw)
    if "joints" not in values or "capsules" not in values:
        raise ValueError("模型必须明确提供 joints 和 capsules")
    raw_joints = values.pop("joints")
    if not isinstance(raw_joints, list) or not raw_joints:
        raise ValueError("joints 必须是非空数组")
    if any(not isinstance(j, dict) or "joint_type" not in j for j in raw_joints):
        raise ValueError("每个关节必须显式填写 joint_type")
    joints = [MotorJoint(**j) for j in raw_joints]
    values["capsules"] = [Capsule(**c) for c in values["capsules"]]
    return build_model(joints, **values)


def load_model(path):
    """读取 UTF-8 JSON；所有单位与 build_model 一致。"""
    return model_from_dict(json.loads(Path(path).read_text(encoding="utf-8-sig")))


def model_to_dict(model):
    """导出可回读的通用模型，不包含 GUI 状态或串口连接信息。"""
    pose = model.kinematics.forward(np.zeros(model.dof))
    return dict(joints=[asdict(j) for j in model.joints],
                tool_position_m=pose.position.tolist(), tool_quaternion_xyzw=pose.quaternion_xyzw.tolist(),
                capsules=[dict(name=c.name, joint_count=c.joint_count, start=c.start.tolist(),
                               end=c.end.tolist(), radius=c.radius) for c in model.collision.capsules],
                allowed_pairs=[list(pair) for pair in sorted(model.collision.allowed_pairs)],
                safety_margin_m=model.collision.safety_margin,
                motor_coupling=model.motor_coupling.tolist(),
                motor_bounds_deg=None if model.motor_bounds_deg is None else model.motor_bounds_deg.tolist())
