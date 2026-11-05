"""输入检查工具：复制数组、验证形状/有限值、单位方向和关节类型。"""

import numpy as np


def finite_vector(value, size: int, name: str) -> np.ndarray:
    """复制输入为浮点数组，并要求原始形状是一维、长度为 size。

    本函数不会把二维数组展平成一维。位置 [x, y, z] 和关节变量
    [q1, ..., qN] 都可用本函数检查，但 Nx3 轴上点数组不能传入。

    :param value: 列表或一维数组，例如位置 [x, y, z]。
    :param size: 要求的元素数量，必须与数组长度一致。
    :param name: 错误信息中使用的数据名称。
    :return: 形状为 (size,) 的独立 NumPy 浮点数组。
    :raises ValueError: 形状不符、包含 NaN/无穷大或不能转换成数字。

    Example Input / Output:
        >>> finite_vector([1, 2, 3], 3, "position")
        array([1., 2., 3.])
        >>> finite_vector([1, 2], 3, "position")
        Traceback (most recent call last):
            ...
        ValueError: position must contain 3 finite values
        >>> finite_vector([[1, 2, 3]], 3, "position")
        Traceback (most recent call last):
            ...
        ValueError: position must contain 3 finite values
    """
    vector = np.array(value, dtype=float, copy=True)
    if vector.shape != (size,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain {size} finite values")
    return vector


def finite_matrix(value, shape: tuple[int | None, int | None], name: str) -> np.ndarray:
    """复制输入为二维浮点数组，检查形状、非空以及所有元素是否有限。

    :param value: 二维列表或数组；不改变维度，也不自动转置。
    :param shape: (行数, 列数)；None 表示该维度可以是任意正整数。
        例如 (4, 4) 检查矩阵大小，(6, None) 检查 6xN 螺旋轴矩阵。
    :param name: 报错时显示的数据名称。
    :return: 已检查的二维数组副本。
    :raises ValueError: 不是二维、大小不符、为空或包含 NaN/无穷大。
        本函数只检查数值和形状，不证明它是有效的旋转或齐次变换。

    Example Input / Output:
        >>> finite_matrix([[1, 2], [3, 4]], (2, None), "matrix")
        array([[1., 2.],
               [3., 4.]])
        >>> finite_matrix([1, 2], (2, 1), "matrix")
        Traceback (most recent call last):
            ...
        ValueError: matrix must be a finite nonempty matrix with shape (2, 1)
    """
    matrix = np.array(value, dtype=float, copy=True)
    if (matrix.ndim != 2 or matrix.size == 0 or not np.all(np.isfinite(matrix))
            or any(expected is not None and actual != expected
                   for actual, expected in zip(matrix.shape, shape))):
        raise ValueError(f"{name} must be a finite nonempty matrix with shape {shape}")
    return matrix


def unit_vectors(value, count: int, name: str) -> np.ndarray:
    """检查 count 个三维单位向量，按行排列；不自动归一化输入。

    :param value: 形状为 (count, 3) 的数组或二维列表。
    :param count: 需要的方向向量数量。
    :param name: 错误信息中使用的名称。
    :return: 已验证的浮点数组副本。
    :raises ValueError: 形状/有限值不符，或某个方向的模长不接近 1。

    Example Input / Output:
        >>> unit_vectors([[0, 0, 1], [1, 0, 0]], 2, "axes")
        array([[0., 0., 1.],
               [1., 0., 0.]])
        >>> unit_vectors([[0, 0, 2]], 1, "axes")
        Traceback (most recent call last):
            ...
        ValueError: axes must contain unit vectors
    """
    vectors = finite_matrix(value, (count, 3), name)
    if not np.allclose(np.linalg.norm(vectors, axis=1), 1.0, atol=1e-8, rtol=0):
        raise ValueError(f"{name} must contain unit vectors")
    return vectors


def validate_joint_types(joint_types) -> tuple[str, ...]:
    """检查非空的关节类型序列，只允许 hinge 和 slide。

    :param joint_types: 类型列表或元组；每项是 "hinge" 或 "slide"。
    :return: 验证后的元组，保持输入中的运动链顺序。
    :raises ValueError: 传入单个字符串、空序列、非序列或不支持的类型。

    Example Input / Output:
        >>> validate_joint_types(["hinge", "slide"])
        ('hinge', 'slide')
        >>> validate_joint_types(["ball"])
        Traceback (most recent call last):
            ...
        ValueError: unsupported joint type: ball; expected hinge or slide
    """
    try:
        types = tuple(joint_types)
    except TypeError as exc:
        raise ValueError("joint_types must be a nonempty sequence of hinge/slide strings") from exc
    if not types or isinstance(joint_types, str):
        raise ValueError("joint_types must be a nonempty sequence of hinge/slide strings")
    for joint_type in types:
        if not isinstance(joint_type, str) or joint_type not in ("hinge", "slide"):
            raise ValueError(f"unsupported joint type: {joint_type}; expected hinge or slide")
    return types


def rigid_transform(value, name="transform") -> np.ndarray:
    """检查转换矩阵是否为刚性变换(4x4 齐次矩阵，旋转部分为正交矩阵，行列式为 1)"""
    matrix = finite_matrix(value, (4, 4), name)
    rotation = matrix[:3, :3]
    if (not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9, rtol=0)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-8, rtol=0)):
        raise ValueError(f"{name} must be a rigid transform with a proper rotation")
    return matrix
