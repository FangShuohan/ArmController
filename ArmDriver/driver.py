"""正逆运动学与路径解算：只输出电机 deg，不生成速度/时间轨迹。"""
from dataclasses import dataclass
from time import monotonic, sleep

import numpy as np
from scipy.optimize import least_squares

from ArmController import IKSettings, IKSolver, JointPathPlanner, PhaseSegment, Pose
from ArmController.geometry import error_norms
from ArmController.validation import finite_vector
from .model import ArmModel


class ExecutionError(RuntimeError):
    """反馈/通信/协同执行失败；已尝试向 MCU 请求停止。"""


@dataclass(frozen=True)
class MotorPlan:
    """电机顺序与 model.motor_ids 一致；所有角度单位均为 deg。

    waypoints_deg 含起点，target_deg 是绝对终点，delta_deg 是总转角。
    segment_deltas_deg 是逐段转角；UART 发绝对终点，避免丢包累加增量。
    """
    motor_ids: tuple[int, ...]
    waypoints_deg: tuple[tuple[float, ...], ...]

    @property
    def target_deg(self):
        return self.waypoints_deg[-1]

    @property
    def delta_deg(self):
        return tuple(b - a for a, b in zip(self.waypoints_deg[0], self.target_deg))

    @property
    def segment_deltas_deg(self):
        return tuple(tuple(b - a for a, b in zip(start, end))
                     for start, end in zip(self.waypoints_deg[:-1], self.waypoints_deg[1:]))


@dataclass(frozen=True)
class MotorTarget:
    """单向发送的绝对目标，单位 deg；没有实际到位或路径安全承诺。

    current_motor_deg 是调用者提供的已知/估计起点，不能视为反馈。
    只支持一个目标，避免把绕行路点无等待地发给只保留最新目标的固件。
    """
    motor_ids: tuple[int, ...]
    current_motor_deg: tuple[float, ...]
    target_deg: tuple[float, ...]

    def __post_init__(self):
        ids = tuple(self.motor_ids)
        if (not ids or any(isinstance(i, bool) or not isinstance(i, int) or not 1 <= i <= 255 for i in ids)
                or len(set(ids)) != len(ids)):
            raise ValueError("motor_ids must contain unique integers in [1,255]")
        object.__setattr__(self, "motor_ids", ids)
        for name in ("current_motor_deg", "target_deg"):
            object.__setattr__(self, name, tuple(float(v) for v in
                finite_vector(getattr(self, name), len(ids), name)))

    @property
    def delta_deg(self):
        return tuple(b - a for a, b in zip(self.current_motor_deg, self.target_deg))


@dataclass(frozen=True)
class MotorPairPreference:
    """逆解的可选绝对电机角度偏好；不修改显式关节目标或串口帧。

    满足原任务容差的候选中，优先 |m1-m2| 或 |m1+m2| 小于容差的结果。
    motor_ids 可来自任意构型；零偏和传动仍由模型定义，不重新解释角度。
    """
    motor_ids: tuple[int, int]
    tolerance_deg: float = 0.001

    def __post_init__(self):
        ids = tuple(self.motor_ids)
        if (len(ids) != 2 or ids[0] == ids[1] or
                any(isinstance(i, bool) or not isinstance(i, int) or not 1 <= i <= 255 for i in ids)):
            raise ValueError("preference requires two distinct motor IDs in [1,255]")
        tolerance = self.tolerance_deg
        if (isinstance(tolerance, bool) or not isinstance(tolerance, (int, float)) or
                not np.isfinite(tolerance) or tolerance <= 0):
            raise ValueError("preference tolerance_deg must be positive and finite")
        object.__setattr__(self, "motor_ids", ids)

    def indices(self, motor_ids):
        ids = tuple(motor_ids)
        if not set(self.motor_ids).issubset(ids):
            raise ValueError("preferred motor IDs must belong to the model")
        return tuple(ids.index(i) for i in self.motor_ids)

    def relation(self, motor_ids, angles_deg):
        i, j = self.indices(motor_ids)
        values = finite_vector(angles_deg, len(motor_ids), "motor angles deg")
        if abs(values[i] - values[j]) <= self.tolerance_deg:
            return "equal"
        if abs(values[i] + values[j]) <= self.tolerance_deg:
            return "opposite"
        return "combined"


class ArmDriver:
    """model 是 build_model() 结果；通信通过 execute(plan, uart) 显式传入。

    forward/current_motor_deg 都读绝对电机 deg；工具位置 m、四元数 xyzw。
    内部自动转换 rad/m，用户无需在代码里手工换算电机角度。
    motor_pair_preference 只用于 solve_target_pose；旧 solve_pose 路径接口沿用原策略。
    """
    def __init__(self, model, *, ik_settings=None, planner_settings=None, motor_pair_preference=None):
        if not isinstance(model, ArmModel):
            raise TypeError("model must come from build_model")
        if motor_pair_preference is not None:
            if not isinstance(motor_pair_preference, MotorPairPreference):
                raise TypeError("motor_pair_preference must be MotorPairPreference or None")
            motor_pair_preference.indices(model.motor_ids)
        self.model = model
        self.motor_pair_preference = motor_pair_preference
        self.ik_settings = ik_settings or IKSettings()
        self.planner = JointPathPlanner(model.collision, planner_settings)

    def forward(self, motor_positions_deg):
        q = self.model.motor_to_joints(motor_positions_deg)
        return self.model.kinematics.forward(q)

    def _motor_plan(self, joint_path):
        return MotorPlan(self.model.motor_ids, tuple(
            tuple(float(v) for v in self.model.joints_to_motor(q)) for q in joint_path))

    def solve_pose(self, position_m, quaternion_xyzw=None, *, current_motor_deg):
        """求逆解与已验证路径，返回每个电机绝对目标及需要转过的 deg。

        task_mode=position 时可省略四元数；其他模式必须给出完整姿态。
        这里只计算，不连接串口，不发送指令。
        """
        if quaternion_xyzw is None:
            if self.ik_settings.task_mode != "position":
                raise ValueError("quaternion_xyzw is required unless task_mode=position")
            quaternion_xyzw = [0, 0, 0, 1]
        q = self.model.motor_to_joints(current_motor_deg)
        path = self.planner.plan_pose(q, Pose(position_m, quaternion_xyzw), self.ik_settings)
        return self._motor_plan(path)

    def solve_joints(self, joint_positions, *, current_motor_deg):
        """直接关节目标：hinge 为 deg，slide 为 m；仍检查/规划整段路径。"""
        start = self.model.motor_to_joints(current_motor_deg)
        goal = self.model.user_to_joints(joint_positions)
        return self._motor_plan(self.planner.plan(start, goal))

    def _target(self, current_motor_deg, goal):
        start = finite_vector(current_motor_deg, self.model.dof, "current_motor_deg")
        self.model.collision.require_valid(self.model.motor_to_joints(start))
        self.model.collision.require_valid(goal)
        return MotorTarget(self.model.motor_ids, tuple(start), tuple(self.model.joints_to_motor(goal)))

    def solve_target_joints(self, joint_positions, *, current_motor_deg):
        """仅解算端点：hinge/deg、slide/m；用于无反馈、独立单轴 S 的设备。"""
        return self._target(current_motor_deg, self.model.user_to_joints(joint_positions))

    def solve_target_pose(self, position_m, quaternion_xyzw=None, *, current_motor_deg):
        """求目标位姿并检查端点；可选电机等角/反角偏好不能放宽任务容差。

        偏好解不存在或不通过校验时，回退到原来的有效组合解。
        这只是候选选择，不代表对实际机构精度或运动路径的保证。
        """
        start = self.model.motor_to_joints(current_motor_deg)
        self.model.collision.require_valid(start)
        if quaternion_xyzw is None:
            if self.ik_settings.task_mode != "position":
                raise ValueError("quaternion_xyzw is required unless task_mode=position")
            quaternion_xyzw = [0, 0, 0, 1]
        solver = IKSolver(self.model.kinematics, self.model.collision.lower,
                          self.model.collision.upper, start, mode="multi-start", settings=self.ik_settings)
        pose = Pose(position_m, quaternion_xyzw)
        candidates = solver.candidate_solutions(pose, start)
        targets = []
        for candidate in candidates:
            try:
                targets.append((self._target(current_motor_deg, candidate.joints), candidate.cost))
            except ValueError:
                continue
        preference = self.motor_pair_preference
        if preference is not None:
            symmetric = [row for row in targets
                         if preference.relation(row[0].motor_ids, row[0].target_deg) != "combined"]
            if not symmetric:
                symmetric = self._symmetric_pose_targets(solver, pose, start, current_motor_deg, candidates)
            if symmetric:
                return min(symmetric, key=lambda row: row[1])[0]
        if targets:
            return targets[0][0]
        raise ValueError("all IK targets violate collision or motor bounds")

    def _symmetric_pose_targets(self, solver, pose, start, current_motor_deg, candidates):
        """额外尝试等角/反角解；仍以原位置/姿态容差、限位和碰撞为准。

        只影响候选生成，不将偏好残差混入最终任务的通过条件。
        对物理电机线性关系做精确投影后，再完整校验一次 FK。
        """
        preference = self.motor_pair_preference
        i, j = preference.indices(self.model.motor_ids)
        seeds = [start, *[c.joints for c in candidates[:2]]]
        targets = []
        settings = solver.settings
        for sign in (1, -1):
            # m = zero + C @ (scale * q)，所以等角/反角均为 q 的线性条件。
            coefficient = (self.model.motor_coupling[i] - sign * self.model.motor_coupling[j]) * self.model.scale
            bias = self.model.zero_deg[i] - sign * self.model.zero_deg[j]
            pivot = int(np.argmax(np.abs(coefficient)))

            def residual(q):
                pair_error_rad = np.deg2rad(coefficient @ q + bias)
                return np.r_[solver.residual(q, pose), settings.orientation_weight * pair_error_rad]

            for seed in seeds:
                result = least_squares(residual, seed, bounds=(solver.lower, solver.upper),
                                       x_scale=solver.upper - solver.lower, max_nfev=settings.max_evaluations,
                                       xtol=settings.optimizer_tolerance, ftol=settings.optimizer_tolerance,
                                       gtol=settings.optimizer_tolerance)
                q = result.x.copy()
                q[pivot] = 0
                q[pivot] = -(coefficient @ q + bias) / coefficient[pivot]
                try:
                    target = self._target(current_motor_deg, q)
                except ValueError:
                    continue
                position_error, orientation_error = error_norms(self.model.kinematics.forward(q), pose)
                if position_error > settings.position_tolerance:
                    continue
                if settings.task_mode != "position" and orientation_error > settings.orientation_tolerance:
                    continue
                if preference.relation(target.motor_ids, target.target_deg) == "combined":
                    continue
                targets.append((target, solver.candidate_cost(q, start)))
        return targets

    def send_target(self, target, sender, *, repeat_for_s=0.0, period_s=0.05, cancel_event=None):
        """单向发送并返回写出的帧数，不接收、不等待到位、不自动执行路点。

        repeat_for_s 只重复同一个绝对目标，供链路重同步/续 watchdog 使用。
        不是运动时长；关闭/中断发送也不是停止机械臂。
        """
        if not isinstance(target, MotorTarget) or target.motor_ids != self.model.motor_ids:
            raise ValueError("send_target requires a MotorTarget for this model, not a path")
        self._target(target.current_motor_deg, self.model.motor_to_joints(target.target_deg))
        # 对将在线上发送的 float32 值复核限位和端点碰撞，避免量化后越界。
        quantized = sender.quantize_angles(target.motor_ids, target.target_deg)
        self.model.collision.require_valid(self.model.motor_to_joints(quantized))
        return sender.send_angles(target.motor_ids, quantized,
                                  repeat_for_s=repeat_for_s, period_s=period_s, cancel_event=cancel_event)

    def _check_feedback(self, feedback):
        if feedback.motor_ids != self.model.motor_ids:
            raise ValueError("feedback motor order/IDs do not match model")
        if not feedback.phase_sync:
            raise ValueError("STM32 must support common-phase multi-axis execution")
        if feedback.state == "fault":
            raise ValueError("STM32 reports a fault")
        q = self.model.motor_to_joints(feedback.position_deg)
        self.model.collision.require_valid(q)
        return q

    def _require_stopped(self, feedback):
        self._check_feedback(feedback)
        if (feedback.state not in ("idle", "stopped")
                or np.any(np.abs(feedback.velocity_deg_s) > self.model.stopped_deg_s)):
            raise ValueError("arm must be stopped")

    def execute(self, plan, uart, *, timeout_s=60.0, poll_s=0.01):
        """逐段提交给 STM32 的共同进度执行器，反馈确认停稳后才发下一段。

        timeout 是整个动作的截止时间，不是规划时长。计算器不预测 MCU 速度。
        必须使用新协议固件；单轴独立 S 曲线固件不能宣称 phase_sync=true。
        """
        if (isinstance(timeout_s, bool) or not np.isfinite(timeout_s) or timeout_s <= 0
                or isinstance(poll_s, bool) or not np.isfinite(poll_s) or poll_s <= 0):
            raise ValueError("timeout_s and poll_s must be positive and finite")
        if not isinstance(plan, MotorPlan) or plan.motor_ids != self.model.motor_ids or not plan.waypoints_deg:
            raise ValueError("plan must contain this model's motor IDs and waypoints")
        # 输入非法时不发送；重新复核每条边，防止执行外部构造的未验证路径。
        joint_path = [self.model.motor_to_joints(row) for row in plan.waypoints_deg]
        for q in joint_path:
            self.model.collision.require_valid(q)
        for first, last in zip(joint_path[:-1], joint_path[1:]):
            self.model.collision.require_edge(first, last)
        deadline = monotonic() + timeout_s
        try:
            feedback = uart.read_feedback()
            self._require_stopped(feedback)
            if np.any(np.abs(np.array(feedback.position_deg) - plan.waypoints_deg[0]) > self.model.tolerance_deg):
                raise ValueError("actual start changed; solve again from fresh feedback")
            tolerance_q = self.model.joint_tolerance
            for target in plan.waypoints_deg[1:]:
                if monotonic() >= deadline:
                    raise TimeoutError("motion timed out")
                actual_start = self._check_feedback(feedback)
                goal = self.model.motor_to_joints(target)
                self.model.collision.require_edge(actual_start, goal)
                segment = PhaseSegment(actual_start, goal, tolerance_q)
                motion_seq = uart.move_segment(
                    self.model.motor_ids, feedback.position_deg, target,
                    self.model.tolerance_deg, self.model.stopped_deg_s,
                )
                phase = 0.0
                while True:
                    if monotonic() >= deadline:
                        raise TimeoutError("motion timed out")
                    feedback = uart.read_feedback()
                    measured = self._check_feedback(feedback)
                    if feedback.motion_sequence != motion_seq:
                        raise ValueError("MCU feedback refers to a different motion")
                    if feedback.phase < phase:
                        raise ValueError("MCU common phase moved backwards")
                    segment.validate(measured, feedback.phase)
                    phase = feedback.phase
                    reached = np.all(np.abs(np.array(feedback.position_deg) - target) <= self.model.tolerance_deg)
                    stopped = np.all(np.abs(feedback.velocity_deg_s) <= self.model.stopped_deg_s)
                    if feedback.state == "idle" and phase == 1.0 and reached and stopped:
                        break
                    if feedback.state != "moving":
                        raise ValueError("MCU stopped without completing the segment")
                    sleep(min(poll_s, max(0.0, deadline - monotonic())))
            return feedback
        except Exception as exc:
            stop_error = None
            try:
                uart.stop()
            except Exception as failure:
                stop_error = str(failure)
            suffix = "" if stop_error is None else f"; STOP failed: {stop_error}"
            raise ExecutionError(str(exc) + suffix) from exc
