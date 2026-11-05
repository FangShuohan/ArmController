# ArmController：通用串联机械臂工具

本目录只放与具体机械臂、仿真和通信无关的算法。内部单位为位置 m、旋转关节 rad、伸缩关节 m，四元数顺序 xyzw。依赖 NumPy、SciPy 和仓库内的 ModernRobotics（Python 3.10+）：

```bash
python -m pip install numpy scipy
```

ModernRobotics 是 NxRLab 的第三方库（Copyright (c) 2019 NxRLab），源码随仓库保存在 [ModernRobotics](../ModernRobotics/) 目录；完整 [MIT 许可证](../ModernRobotics/LICENSE) 和原版权声明均原样保留。直接克隆本仓库即可使用，无需单独 pip 安装 modern_robotics 或初始化子模块。来源和版本见 [第三方声明](../THIRD_PARTY_NOTICES.md)。

## 逐文件审查结果

| 工具 | 实现逻辑与适用范围 | 审查处理 |
| --- | --- | --- |
| validation.py | 保留输入维度，复制/验证有限向量和矩阵、单位轴和类型 | 单位轴采用绝对误差判断；新增 SE(3) 矩阵校验 |
| geometry.py | Pose 使用单位四元数；相对旋转向量而非直接减四元数 | 拒绝不正交、反射、错误齐次末行，避免 SciPy 静默修复错误旋转 |
| kinematics.py | POE 指数积 FK，前缀变换用于零位几何；expm 保留小角微分 | 验证 M 为刚体变换；M、S、预计算 twists 只读，避免缓存与模型不一致 |
| task_space.py | 从轴线构造旋转闭包，保守压缩恒定位置/姿态方向 | 逻辑保留；不把单一构型雅可比秩当作整臂任务维数 |
| inverse_kinematics.py | 有界 least_squares，多初值候选、完整误差复核、归一化位移评分 | 增加全部 IKSettings 入口检查；拒绝越限当前反馈，不再静默裁剪 |
| collision.py | 零位胶囊体按 POE 前缀移动；间距叠加误差包络；整段自适应细分 | 保留保守运动界；缓存几何参数只读；删除依赖 ArmDriver 的示例 |
| path_planning.py | 归一化关节 RRT-Connect；树边、缩短和候选逆解均走同一检查器 | 逻辑保留；删除依赖 ArmDriver 的示例 |
| trajectory.py | 共同单调五次进度，各路点停稳，按最大导数分配时间 | 是通用可选工具，保留；**新驱动不使用它**，也不具备显式 jerk 限值 |
| coordination.py | PhaseSegment 检查全体关节遵循共同进度的直线路径 | 新增；仅几何与反馈偏差检查，不做时间/速度规划 |
| __init__.py | 显式公共导出 | 仅导出通用工具 |

本目录只保留通用算法；具体机械臂模型配置见 ArmDriver/arm_config.json。

## 边界和限制

FK 在给定轴线模型上确定；IK/RRT 是有限预算数值搜索，不保证全局最优或最短路径，失败不证明物理无解。auto 压缩固定方向后仍检查完整目标误差；position 模式明确忽略姿态。

路径检查证明范围是所配置胶囊体和误差包络中的理想关节直线路径，不能自动覆盖未建模障碍物、线缆、力矩、制动距离或受力变形。allowed_pairs 是整对几何不互检，必须按真实允许接触区域谨慎划分。

同时到达不保证路径相同。若用下位机执行已检查的关节边，必须使用共同进度 q=start+s×delta，或另行验证真实插值。PhaseSegment 按 MCU 报告的 s 和实测关节位置检查偏差；S 曲线时间规划放在 STM32。

## 使用与回归

用户接入入口见 [ArmDriver](../ArmDriver/README.md)。独立算法可从本包直接导入 Pose、POEKinematics、IKSolver、SelfCollisionModel、JointPathPlanner、PhaseSegment；不需要导入驱动。

在包含 tests 的完整开发工作区可运行 python -B -m unittest discover -s tests；本仓库不包含该测试目录。开发回归包含 FK 与独立 MuJoCo 对比、小角微分、四元数符号、有限值与 SE(3) 检查、多类型/多自由度、路径运动界与误差包络、真实 RRT 绕行和无驱动/无 MuJoCo 的独立运行。
