"""位姿工具：世界坐标、米、弧度，以及 SciPy 的 xyzw 四元数顺序。

文档中 >>> 开头的行是示例输入，随后不带 >>> 的行是实际输出。
数值示例用 round/np.round 消除浮点计算产生的微小显示差异。
"""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

import modern_robotics as mr

from .validation import finite_vector, rigid_transform


@dataclass(frozen=True)
class Pose:
    """末端相对世界坐标系的位姿；四元数自动归一化。"""

    position: np.ndarray
    quaternion_xyzw: np.ndarray

    def __post_init__(self):
        """在创建 Pose 后自动检查位置、归一化四元数并保护数组。

        不需要手动调用。@dataclass 自动生成的构造函数会调用本方法；
        frozen=True 和只读数组用于防止已保存的目标被意外修改。

        :return: None；更新当前对象的位置和四元数属性。
        :raises ValueError: 位置无效或四元数无效/模长接近零。

        Example Input / Output:
            >>> pose = Pose([1, 2, 3], [0, 0, 0, 2])
            >>> pose.quaternion_xyzw
            array([0., 0., 0., 1.])
            >>> pose.position.flags.writeable
            False
        """
        position = finite_vector(self.position, 3, "position")
        quaternion = finite_vector(self.quaternion_xyzw, 4, "quaternion_xyzw")
        norm = np.linalg.norm(quaternion)
        if not np.isfinite(norm) or norm < 1e-12:
            raise ValueError("quaternion_xyzw must have a finite, nonzero norm")
        quaternion /= norm
        position.setflags(write=False)
        quaternion.setflags(write=False)
        object.__setattr__(self, "position", position)
        object.__setattr__(self, "quaternion_xyzw", quaternion)

    @classmethod
    def from_matrix(cls, matrix: np.ndarray) -> "Pose":
        """根据齐次变换矩阵创建新的Pose对象(位置加四元数)

        :param matrix: 有效的 4x4 齐次变换；左上角为旋转，最后一列为位置。
        :return: 新的 Pose 对象；单位变换的四元数是 [0, 0, 0, 1]。

        Example Input / Output:
            >>> matrix = np.eye(4)
            >>> matrix[:3, 3] = [1, 2, 3]
            >>> Pose.from_matrix(matrix).as_tuple()
            (array([1., 2., 3.]), array([0., 0., 0., 1.]))
        """
        rotation, position = mr.TransToRp(rigid_transform(matrix))
        return cls(position, Rotation.from_matrix(rotation).as_quat())

    def as_matrix(self) -> np.ndarray:
        """将位姿转换为齐次变换矩阵。

        :return: 形状为 (4, 4) 的新数组。

        Example Input / Output:
            >>> Pose([1, 2, 3], [0, 0, 0, 1]).as_matrix()
            array([[1., 0., 0., 1.],
                   [0., 1., 0., 2.],
                   [0., 0., 1., 3.],
                   [0., 0., 0., 1.]])
        """
        return mr.RpToTrans(Rotation.from_quat(self.quaternion_xyzw).as_matrix(), self.position)

    def as_tuple(self) -> tuple[np.ndarray, np.ndarray]:
        """取出位置与四元数的副本，方便外部程序读取。

        :return: (position, quaternion_xyzw)，两个独立可写数组。
            修改返回数组不会改变 Pose 内部保存的数据。

        Example Input / Output:
            >>> pose = Pose([1, 2, 3], [0, 0, 0, 1])
            >>> position, quaternion = pose.as_tuple()
            >>> position[0] = 99
            >>> pose.position
            array([1., 2., 3.])
        """
        return self.position.copy(), self.quaternion_xyzw.copy()


def pose_error(actual: Pose, target: Pose) -> tuple[np.ndarray, np.ndarray]:
    """返回位置误差 (m) 和世界坐标下的姿态旋转向量 (rad)。

    e_p = p_actual - p_target
    e_R = Log(R_target R_actual^T)^vee
    用相对旋转而非四元数分量相减，保证 q 和 -q 得到相同误差。

    :param actual: 当前位姿。
    :param target: 目标位姿，与 actual 使用同一坐标系。
    :return: (位置误差向量, 姿态误差旋转向量)，形状分别为 (3,)、(3,)。
        位置符号是实际减目标；姿态描述从实际旋转到目标的相对旋转。

    Example Input / Output:
        >>> actual = Pose([0, 0, 0], [0, 0, 0, 1])
        >>> target = Pose([1, 0, 0], Rotation.from_euler("z", 90, degrees=True).as_quat())
        >>> position_error, rotation_error = pose_error(actual, target)
        >>> position_error.tolist()
        [-1.0, 0.0, 0.0]
        >>> np.round(rotation_error, 4).tolist()
        [0.0, 0.0, 1.5708]
    """
    actual_rotation = Rotation.from_quat(actual.quaternion_xyzw)
    target_rotation = Rotation.from_quat(target.quaternion_xyzw)
    rotation_error = (target_rotation * actual_rotation.inv()).as_rotvec()
    return actual.position - target.position, rotation_error


def error_norms(actual: Pose, target: Pose) -> tuple[float, float]:
    """将位姿误差向量转换为两个非负标量，用于判断是否达到容差。

    :param actual: 当前位姿。
    :param target: 目标位姿。
    :return: (位置误差的欧氏长度/m, 最短相对旋转角/rad)。

    Example Input / Output:
        >>> actual = Pose([0, 0, 0], [0, 0, 0, 1])
        >>> target = Pose([3, 4, 0], [0, 0, 0, 1])
        >>> error_norms(actual, target)
        (5.0, 0.0)
    """
    position_error, rotation_error = pose_error(actual, target)
    return float(np.linalg.norm(position_error)), float(np.linalg.norm(rotation_error))
