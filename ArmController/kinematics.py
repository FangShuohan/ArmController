"""POE 正运动学，参考 ModernRobotics 的 FKinSpace。

直接复用 ScrewToAxis 和 VecTose3。仅矩阵指数使用 scipy.linalg.expm，
避免 MR 的 MatrixExp6 在极小转角处的 NearZero 分支影响数值微分。
本模块不依赖 MuJoCo，可用于独立的运动学计算。
"""

import numpy as np
from scipy.linalg import expm

import modern_robotics as mr

from .geometry import Pose
from .validation import finite_matrix, finite_vector, rigid_transform, unit_vectors, validate_joint_types


class POEKinematics:
    def __init__(self, zero_transform: np.ndarray, screw_axes: np.ndarray):
        """保存零位末端变换 M、空间螺旋轴 S，并预计算螺旋轴矩阵。

        :param zero_transform: 全零关节坐标下的 4x4 末端变换 M。
        :param screw_axes: 6xN 矩阵；每列是 [omega_x,y,z; v_x,y,z]。
        :return: 构造函数不返回值；得到可调用 forward 的模型对象。
        :raises ValueError: 矩阵形状不符合要求或包含非有限值。

        Example Input / Output:
            >>> screws = np.array([[0, 0, 1, 0, 0, 0]]).T
            >>> kinematics = POEKinematics(np.eye(4), screws)
            >>> kinematics.dof
            1
            >>> kinematics.twists[0]
            array([[ 0., -1.,  0.,  0.],
                   [ 1.,  0., -0.,  0.],
                   [-0.,  0.,  0.,  0.],
                   [ 0.,  0.,  0.,  0.]])
        """
        self.zero_transform = rigid_transform(zero_transform, "zero_transform")
        self.screw_axes = finite_matrix(screw_axes, (6, None), "screw_axes")
        # MR 约定每一列为一个 S_i=[omega_i; v_i]，预计算对应的 [S_i]。
        self.dof = self.screw_axes.shape[1]
        self.twists = tuple(mr.VecTose3(screw) for screw in self.screw_axes.T)
        for value in (self.zero_transform, self.screw_axes, *self.twists):
            value.setflags(write=False)

    @classmethod
    def from_joints(cls, axes, anchors, joint_types, zero_pose: Pose) -> "POEKinematics":
        """根据关节类型构造混合串联机械臂的空间螺旋轴。

        hinge: S=[omega; -omega x a]；slide: S=[0; d]。
        各轴方向必须是同一零位世界坐标下的单位向量。

        :param axes: Nx3 轴方向数组，在全零关节变量姿态的世界坐标中表示。
        :param anchors: Nx3 轴上一点的位置数组，单位 m，也必须来自全零姿态。
            这是几何参数，不是 forward_matrix 的关节变量 q。
            任意关节原点只要位于对应轴上即可使用；伸缩关节不使用该点。
        :param joint_types: N 个 "hinge" 或 "slide" 字符串，按运动链顺序排列。
        :param zero_pose: 关节坐标均为 0 时的末端位姿。
        :return: 螺旋轴矩阵形状为 6xN 的 POEKinematics。
        :raises ValueError: 类型不支持、数组形状不符或轴不是单位向量。

        Example Input / Output:
            >>> pose = Pose([0, 0, 0], [0, 0, 0, 1])
            >>> k = POEKinematics.from_joints([[0, 0, 1], [1, 0, 0]],
            ...     [[0, 0, 0], [0, 0, 0]], ["hinge", "slide"], pose)
            >>> k.screw_axes.T.tolist()
            [[0.0, 0.0, 1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0, 0.0, 0.0]]
            >>> np.round(k.forward([np.pi / 2, 0.2]).position, 4).tolist()
            [0.0, 0.2, 0.0]
        """
        joint_types = validate_joint_types(joint_types)
        count = len(joint_types)
        axes = unit_vectors(axes, count, "axes")
        anchors = finite_matrix(anchors, (count, 3), "anchors")
        screws = []
        for axis, anchor, joint_type in zip(axes, anchors, joint_types):
            if joint_type == "hinge":
                screws.append(mr.ScrewToAxis(anchor, axis, 0))
            else:  # validate_joint_types 已保证剩下的类型只能是 slide。
                screws.append(np.r_[np.zeros(3), axis])
        return cls(zero_pose.as_matrix(), np.column_stack(screws))

    def forward_matrix(self, joint_variables) -> np.ndarray:
        """使用指数积公式计算末端齐次变换。

        T(q) = exp([S1]q1) ... exp([Sn]qn) M。
        旋转关节变量单位 rad，伸缩关节变量单位 m。

        :param joint_variables: 形状 (dof,) 的一维关节变量 q=[q1,...,qN]。
            每个关节对应一个标量：hinge 是角度/rad，slide 是伸缩量/m。
            不是各关节原点的 XYZ 空间位置，不是形状 (dof, 3) 的 anchors。
        :return: 4x4 末端变换，相对于构造模型时的空间坐标系。
        :raises ValueError: 关节数量不符或关节值不是有限数。

        Example Input / Output:
            >>> zero_pose = Pose([1, 0, 0], [0, 0, 0, 1])
            >>> kinematics = POEKinematics.from_joints(
            ...     [[0, 0, 1]], [[0, 0, 0]], ["hinge"], zero_pose)
            >>> np.round(kinematics.forward_matrix([np.pi / 2]), 4)
            array([[ 0., -1.,  0.,  0.],
                   [ 1.,  0.,  0.,  1.],
                   [ 0.,  0.,  1.,  0.],
                   [ 0.,  0.,  0.,  1.]])
        """
        joint_variables = finite_vector(joint_variables, self.dof, "joint_variables")
        transform = np.eye(4)
        for screw_matrix, joint_variable in zip(self.twists, joint_variables):
            transform = transform @ expm(screw_matrix * joint_variable)
        return transform @ self.zero_transform

    def prefix_transforms(self, joint_variables) -> np.ndarray:
        """计算基座及前 k 个关节的 POE 乘积，用于所有连杆的几何变换。

        :param joint_variables: 长度 dof 的关节变量；hinge/rad，slide/m。
        :return: (dof+1, 4, 4) 数组；第 0 项是单位矩阵，第 k 项只含前 k 个指数。
            本函数不乘末端零位矩阵 M，几何端点应在零位空间坐标系中定义。

        Example Input / Output:
            >>> k = POEKinematics(np.eye(4), np.array([[0, 0, 0, 1, 0, 0]]).T)
            >>> k.prefix_transforms([0.2])[:, 0, 3].tolist()
            [0.0, 0.2]
        """
        values = finite_vector(joint_variables, self.dof, "joint_variables")
        transforms = np.repeat(np.eye(4)[None, :, :], self.dof + 1, axis=0)
        for index, (screw, value) in enumerate(zip(self.twists, values)):
            transforms[index + 1] = transforms[index] @ expm(screw * value)
        return transforms

    def forward(self, joint_variables) -> Pose:
        """计算 POE 正运动学，将矩阵结果转成位置与四元数。

        :param joint_variables: 长度为 dof 的一维关节变量；hinge 用 rad，slide 用 m。
        :return: Pose 对象，四元数顺序为 xyzw。

        Example Input / Output:
            >>> zero_pose = Pose([1, 0, 0], [0, 0, 0, 1])
            >>> kinematics = POEKinematics.from_joints(
            ...     [[0, 0, 1]], [[0, 0, 0]], ["hinge"], zero_pose)
            >>> pose = kinematics.forward([np.pi / 2])
            >>> np.round(pose.position, 4).tolist()
            [0.0, 1.0, 0.0]
            >>> np.round(pose.quaternion_xyzw, 4).tolist()
            [0.0, 0.0, 0.7071, 0.7071]
        """
        return Pose.from_matrix(self.forward_matrix(joint_variables))
