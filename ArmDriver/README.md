# ArmController / ArmDriver：从 JSON 建模到实机调试

现在只用一个调试入口：**`python -m ArmDriver`**。在 `arm_config.json` 修改模型，在 GUI 中预览、求逆解和下发串口目标。原 `example_uart_sync.py` 和尺寸参数 JSON 已合并到这条流程并删除；你填写的尺寸和传动参数保存在新的 `arm_config.json`。

ArmController 提供通用数学工具；ArmDriver/model.py 负责模型校验和单位换算，driver.py 负责解算，uart.py 负责通信，debug_gui.py 负责界面。四轴构型和差动关系只在配置中定义，没有写死在通用解算器里。旧 example.py / UARTClient 是另一套双向协议的软件回归入口，不用于当前 app.cpp；默认 `python -m ArmDriver` 已改为启动 GUI。

## 1. 安装环境并启动

在终端执行：

```powershell
git clone https://github.com/FangShuohan/ArmController.git
cd ArmController
conda create -n robotic_arm python=3.11 -y
conda activate robotic_arm
python -m pip install numpy scipy matplotlib pyserial
python -B -m ArmDriver
```

Python 需要 3.10 或更新版本。ModernRobotics 源码已包含在仓库的 [ModernRobotics](../ModernRobotics/) 目录中，无需单独安装或初始化子模块。它由 NxRLab 开发（Copyright (c) 2019 NxRLab），完整 [MIT 许可证](../ModernRobotics/LICENSE) 和原版权声明均原样保留；来源和版本见 [第三方声明](../THIRD_PARTY_NOTICES.md)。

Tk 使用 Anaconda 自带版本；若缺少 tkinter，在该环境执行 `conda install tk`。也能直接运行 `python -B ArmDriver/debug_gui.py`。打开其他配置使用：

```powershell
python -B -m ArmDriver --config ArmDriver/arm_config.json
```

默认读取脚本旁的 `arm_config.json`，不依赖终端当前路径。没有下位机时，也可以完成建模、预览、逆解和发送帧检查。**连接串口本身不会发送运动指令。**

## 2. 先选零位，再把实物尺寸写入 JSON

先复制 `ArmDriver/arm_config.json` 留作备份。下面采用你的机械臂：大臂俯仰 → 小臂伸缩 → 末端俯仰 → 吸盘自转；末端两个物理电机通过差动机构组合控制。

请实际做这几件事：

1. 在图纸或未使能的机构上选可重复识别的参考姿态：大臂水平朝前，小臂完全缩回，末端俯仰与吸盘自转为 0。这个姿态定义所有逻辑关节为 0。
2. 在底座标出坐标系：原点在大臂俯仰轴心正下方；X 朝机械臂伸出方向，Z 向上，Y 按右手系确定。后面所有轴心、工具点和包络端点，都用**这个基座系在零姿态时的坐标**，不要填写逐节局部坐标。
3. 测量零位尺寸。当前 JSON 保留了你填写的值：大臂 0.43 m，缩回小臂 0.113 m，基座轴高 0.07 m，工具长 0.065 m。长度先把 mm 除以 1000。
4. 得到下面的轴心和工具点，在 JSON 中逐一核对。两末端轴相交只是当前示例；实物有偏置就分别测量填写。

| JSON 位置 | 当前值 | 你要测什么 |
| --- | --- | --- |
| `model.joints[0].axis_point_m` | `[0,0,0.07]` | 大臂俯仰轴心 |
| `model.joints[1].axis_point_m` | `[0.43,0,0.07]` | 伸缩轴上的一点 |
| `model.joints[2].axis_point_m` | `[0.543,0,0.07]` | 末端俯仰轴心 |
| `model.joints[3].axis_point_m` | `[0.543,0,0.07]` | 吸盘自转轴心 |
| `model.tool_position_m` | `[0.608,0,0.07]` | 实际希望控制的工具工作点 |

例如改小臂缩回长度时，必须一起改末端两轴心、工具点和相关 capsules 端点。JSON 存完整零位几何，不会根据某个长度字段自动推导其他坐标。

5. 查看下位机调试器中四个物理电机的协议角度，按模型 ID `[1,2,3,4] = [大臂,伸缩,末端1,末端2]` 记录到对应 `zero_deg`。这是**逻辑零姿态时的物理电机读数**，不一定都是 0。当前文件中的 0 尚需标定。模型零位不等于自动回零，也不等于随便选一次上电位置。
6. 测量行程、减速比和伸缩传动，再修改下一节参数。不要把多圈电机角度对 360° 取模。

实物无法水平摆放时可以选其他可重复姿态，但必须重新填写该姿态下全部轴向、轴心、工具位置及工具方向，不能只改工具点。

## 3. JSON 中具体改哪些参数

顶层包含 `model`、`uart` 和可选 `control`。`control` 只设置逆解候选偏好，不改变机构几何或固件协议。`model.joints` 按基座到工具的**逻辑关节顺序**排列，每个关节显式填写 `joint_type`。可用文本编辑器修改，也可在 GUI“模型参数”页编辑。

| 参数 | 填法与单位 |
| --- | --- |
| `name` | 可选显示名称，如“末端相对俯仰”；不改变运动学与协议 |
| `joint_type` | 旋转填 `"hinge"`，伸缩填 `"slide"` |
| `axis_xyz` | 零姿态下基座系中的正方向，必须单位向量；本机俯仰 `[0,-1,0]`、伸缩/吸盘自转 `[1,0,0]` |
| `axis_point_m` | 零姿态下轴上的一点，基座系坐标，m |
| `lower`、`upper` | 逻辑关节限位；hinge 为 deg，slide 为 m；填写机械允许行程 |
| `gear_ratio` | 电机转角/关节转角的倍率；伸缩还要结合每转位移 |
| `direction` | 传动方向，`1` 或 `-1`；几何正轴与实物动作先统一，再标定传动正负 |
| `screw_lead_m` | 伸缩每转位移，m/rev；当前 `0.082`。旋转关节填 `null` |
| `zero_deg` | 物理电机在逻辑零姿态时的协议角度，deg |
| `uncertainty` | 逻辑关节误差预算：hinge 为 deg，slide 为 m，用于碰撞余量 |
| `tolerance_deg`、`stopped_deg_s` | 物理电机容差，供通用校验/旧反馈接口使用；不会发给当前固件 |

内部解算使用 rad/m，**电机目标由驱动层统一转换为 deg**。不要再给串口角度乘 180/π。伸缩示例中 1 mm 对应 `360×0.001/0.082 ≈ 4.390244 deg`。

差动矩阵 `model.motor_coupling` 目前为：

```json
[[1,0,0,0],[0,1,0,0],[0,0,-1,2],[0,0,1,2]]
```

表示末端1=−俯仰+2×吸盘自转、末端2=俯仰+2×吸盘自转：俯仰时两电机反向，自转时两电机同向。根据实机标定，俯仰列为 `[-1,+1]`，自转列为 `[2,2]`：两电机同向各转 180°，吸盘自转 90°。倍率只在矩阵中计入，J4 的 gear_ratio 保持 1，串口发送时不再额外乘 2。 从两电机各自端面观察的顺/逆时针不能直接当作统一的协议正负，需要核对固件 reverse。 映射公式为 `motor_deg = zero_deg + C @ scaled_joint`。矩阵列和传动倍率对应逻辑关节；矩阵行、motor_id、zero_deg 和物理电机上下限对应物理电机。

已打开的 GUI 不会自动重新读取文件：先断开串口，打开修正后的参数文件并校验应用，再重新填写实测电机角度、计算待发目标。旧待发目标和“上次发送目标”的估计起点不可沿用；实测协议角度本身不要手动取反。此次方向修正只在上位机传动矩阵中做，不要同时翻转 J3 的 direction、axis_xyz 或下位机 reverse，否则会重复反向。

`motor_bounds_deg` 是物理电机绝对角度限制。当前伸缩逻辑上限为 0.266 m，每转 0.082 m，零偏为 0、倍率为 1 时对应约 1167.804878 deg，位于物理电机上限 2250 deg 内。逻辑限位和物理限位同时生效，修改零偏或传动后应重新核对。

`tool_quaternion_xyzw` 是工具零位相对于基座的方向，顺序 xyzw；当前同向填 `[0,0,0,1]`。

`capsules` 用零姿态端点和半径描述外形，单位 m。`joint_count=0` 固定在底座，k 表示随前 k 个逻辑关节运动。按 CAD 补齐底座、滑轨、壳体、夹具等；当前包络只是简化示例。`allowed_pairs` 只填实际结构允许接触/重叠的部件对。

串口配置与模型分开：

```json
"uart": {"channel_motor_ids": [3,4,1,2], "baudrate": 115200}
```

通道顺序由当前 app.cpp 数组 `[末端1,末端2,大臂,伸缩]` 决定，不是界面输入顺序。ID 不进入帧；主机根据 ID 重排角度。通用工具支持其他关节数，换固件时需同步修改帧长度与通道顺序。

## 4. 应用参数并做无硬件预览

1. 启动 GUI，在“模型参数”页核对 JSON，点“校验并应用参数”。轴向、限位、矩阵、误差预算或通道映射不合法会显示错误，并保留之前有效模型。
2. 点“保存参数”，保存到使用的 JSON。保存只写文件；编辑后仍需“校验并应用参数”才改变当前模型。外部编辑后，点“打开参数”重新加载。连接串口时先断开再切换/应用模型。
3. 当前参数且零偏为 0 时，工具参考位置应为 `[0.608,0,0.07] m`。黄色箭头为关节正方向，红/绿/蓝为 X/Y/Z；包络线的屏幕宽度不代表实际半径。
4. 到“目标与关节”页，填下面的目标，每次点“参考姿态”后再点“按关节目标预览”。

| 逻辑关节目标 `[大臂deg,伸缩m,末端俯仰deg,吸盘自转deg]` | 电机目标 `[大臂,伸缩,末端1,末端2] deg` |
| --- | --- |
| `[5,0,0,0]` | `[5,0,0,0]` |
| `[0,0.01,0,0]` | `[0,43.902439,0,0]` |
| `[0,0,5,0]` | `[0,0,-5,5]` |
| `[0,0,0,5]` | `[0,0,10,10]` |
| `[20,0.05,-10,15]` | `[20,219.512195,40,20]` |

这些数值以当前倍率、示例差动和零偏 0 为前提；标定后按新配置重新计算。

5. 工具位置填 `X=0.60,Y=0,Z=0.30`，姿态填 `Rx=15,Ry=-10,Rz=0`，不勾“只约束位置”，点“求逆解并预览”。终点应接近 `[0.60,0,0.30]`。

姿态使用固定基座轴 xyz 欧拉角：本机构向上绝对俯仰 10° 对应 Ry=-10°，吸盘自转 15° 对应 Rx=15°。绝对俯仰不等于末端相对大臂的关节角。四自由度机构不能实现任意六维位姿；失败时先检查 Y=0、单位、工作半径和行程。只约束位置时，末端方向由求解器选择。

动画时间只控制屏幕播放。鼠标旋转/缩放，“重置视野”重算范围。输入已知电机角度并点“应用输入角度”只恢复虚拟姿态。“复制当前虚拟姿态的发送帧”只复制十六进制，不发送。

理想路径检查针对逻辑关节直线插值；独立电机 S 曲线可能走不同路径。预览不能证明实际运动全程无碰撞，也不能证明已到位。

## 4a. 吸盘中心不移动，怎么描述自转目标

XYZ 只描述位置。吸盘绕自己的轴转动时，中心坐标可以完全不变，但吸盘上标记的方向发生了变化，这属于**工具姿态**。可在吸盘边缘做一个标记，将标记方向作为自转零角参考；圆形外观的对称性不代表模型没有这个自由度。

两个物理电机同轴是传动机构的特征。JSON 中 J3/J4 描述的是机构输出的俯仰轴和吸盘自转轴，不应把它们改成两根物理电机轴。目前 J4 的零位正轴假定为基座 +X；照片不能确定你的基座坐标与安装方向，实测若不同，修改 J4 的 axis_xyz、轴心和工具零位方向。工具工作点位于自转轴上时，纯自转才不改变该点位置；偏心工作点会绕轴画圆。

**最直接的方法：修改第 4 个逻辑关节。** 在 GUI“目标与关节”页保留当前大臂、伸缩、末端俯仰值，把 J4 改为所需吸盘绝对角度，再到“实机串口”页计算关节待发目标。例如从逻辑零位给 `[0,0,0,30]`，表示吸盘自转到 +30°，当前传动下两末端电机都转到 +60°。若当前 J4=10°，希望再转 30°，目标应填 40°；输入框接收绝对目标，不是增量。

程序中只改变吸盘角度的方法如下。这里用模型生成一个演示起点；连接实机时，将 current 替换成下位机调试器中已确认的实际电机绝对角度。

<!-- walkthrough:rotation -->
```python
import numpy as np
from ArmDriver import load_model, ArmDriver

model = load_model("ArmDriver/arm_config.json")
arm = ArmDriver(model)
current = model.joints_to_motor(model.user_to_joints([20, .05, -10, 0]))
q = model.motor_to_joints(current)
goal = [float(np.rad2deg(v) if j.joint_type == "hinge" else v)
        for j, v in zip(model.joints, q)]
goal[3] += 30  # 在当前吸盘角度上增加 30 deg；其余逻辑关节保持
result = arm.solve_target_joints(goal, current_motor_deg=current)
print("Motor delta deg:", result.delta_deg)  # [0,0,60,60]
print("Tool before:", arm.forward(current).position)
print("Tool after:", arm.forward(result.target_deg).position)  # 位置不变，姿态已变
# 确认后使用 arm.send_target(result, sender) 下发整组目标
```

**通过目标位姿指定自转也可以。** 当前零位工具方向和轴向约定下，GUI 中保留 XYZ 和绝对俯仰，填 `Rx=目标吸盘角度、Ry=−目标绝对俯仰、Rz=0`。例如 `[X,Y,Z]=[0.60,0,0.30]`、`[Rx,Ry,Rz]=[30,-10,0]`，表示该位置、向上俯仰 10°、吸盘自转到 30°。必须取消“只约束位置”，再计算位姿待发目标；只约束位置时，自转角没有被要求，不能靠 XYZ 控制。

上述 Rx 写法依赖当前工具零位方向和机构轴向；其他安装方向应使用完整工具四元数。希望绕当前工具自身 X 轴再转 δ 时，方向组合为 `R_goal = R_current * R_x(δ)`，右乘表示绕工具局部轴，不是绕固定基座 X 轴。

GUI 中工具坐标系的绿/蓝轴会随自转改变；当前轴对称包络中心线可以不变。姿态角表示方向，无法区分相差 360° 的转数。本次按工具 Rx 输入不超过 ±180° 使用；零俯仰、零偏时 Rx=180° 对应两末端电机各 ±360°，正负取决于逆解选择的等价关节解。需要指定转动方向或转数时使用未取模的 J4 绝对目标。当前 J4 配置范围为 ±360°，但纯自转 360° 需要电机 720°，会被物理电机 ±700° 限位拒绝；扩大逻辑范围不会绕过物理限位。

## 4b. 差动组合动作、保持水平与完整末端位姿

GUI 的 J3 是末端**相对大臂的俯仰角**，J4 是吸盘自转角；二者不是电机1/2的角度。示例倍率 1、零偏 0 时：

| J3 俯仰 / deg | J4 自转 / deg | 末端电机1 / deg | 末端电机2 / deg | 意义 |
| --- | --- | --- | --- | --- |
| 30 | 0 | -30 | 30 | 纯俯仰 |
| 0 | 30 | 60 | 60 | 纯自转 |
| 30 | 15 | 0 | 60 | 俯仰与自转叠加，第一台电机角度抵消 |

GUI 现已给逻辑关节加名称，并在关节输入下方逐行显示“物理电机目标 / deg”。输入 J3=-30、J4=0 时应看到 ID3=+30、ID4=-30；J3=0、J4=30 时看到两电机都为 +60。该区域只做换算，不自动发送；待发区仍以独立计算得到的目标为准。

所以只有一台末端电机出现非零目标，在理想模型中可能是有效的组合动作；这不能证明真实机构也有同样的到位精度。你的实机反馈是单电机组合动作容易不准，因此增加下一节的软优先选择。反推为 `俯仰=(m2-m1)/2`、`自转=(m1+m2)/4`；实际应先扣零偏、应用已标定比例。判断是否是纯运动应看逻辑关节变化及物理电机**增量**，而不是只看非零绝对目标。

GUI 默认不勾“只约束位置”，因此同时要求 XYZ 与输入的 Rx/Ry/Rz。初始姿态为零，只改 XYZ 时，求解器会主动补偿末端相对俯仰以满足这份姿态目标。当前模型中 `工具绝对俯仰 = 大臂角 + J3`；目标绝对俯仰为 0 时，J3≈−大臂角，所以工具保持水平。这是完整姿态约束的结果，不是逆解只支持水平。

实际检查方法：位置填 `[0.60,0,0.30]`，分别给姿态 `[0,0,0]` 和 `[0,-20,0]`，不勾“只约束位置”，执行预览。前者工具轴水平，后者向上俯仰 20°；再给 `[30,-20,0]`，会在保持绝对俯仰 20°的同时让吸盘自转到 30°。四自由度无法满足任意六维目标，不可达时应调整目标。

界面现已显示当前约束模式，右侧显示**计算得到的当前工具 Rx/Ry/Rz**，实机待发区也显示逻辑关节与工具姿态结果。左侧姿态框始终是目标输入，关节预览不会偷偷覆盖它；需要沿用当前虚拟姿态时点“从当前虚拟位姿填入目标”，再修改 XYZ。勾选“只约束位置”后姿态输入变灰、不参与计算，工具方向允许变化，不能保证水平。该按钮不读取实机状态，也不发送指令。

“实机串口”页计算/下发目标与三维虚拟状态分开：计算完成不会把三维图自动标成实机已到位。想核对这次逆解的运动与最终方向，点击“预览待发目标（虚拟，不发送）”，动画将从本次填写的起点播放到实际待发的量化角度；待发目标仍可随后下发。不要把原先的水平虚拟姿态误认为刚计算出的目标。

末端位姿包含位置与方向，不只与吸盘自转有关。当前理想模型中：伸缩改变位置；大臂转动改变位置与方向；末端俯仰改变工具方向，并因工具长度而改变工具中心位置；吸盘绕中心轴自转主要改变工具坐标系方向。工具方向同时由大臂角、末端相对俯仰、自转角决定。

本模型把工具 X 轴作为吸盘轴线。“工具轴水平”通常对应吸盘面竖直，适合侧吸；如果你希望的是吸盘面水平，应要求轴线竖直。安装坐标与照片必须通过实物标定对应。Euler 角在特殊姿态存在多解，以完整工具旋转/四元数为准。

## 4c. 优先等角／反角，但保留要求的目标

默认配置现已加入与 `model`、`uart` 同级的这一项：

```json
"control": {
  "motor_pair_preference": {"motor_ids": [3,4], "tolerance_deg": 0.001}
}
```

`motor_ids` 指物理电机 ID，不是串口数组下标；这里是两台末端电机。`tolerance_deg` 只判断两个**绝对协议目标角度**是否相等或互为相反数，包含模型零偏；它不是 IK 的位置/姿态容差，也不是到位精度要求。实机零偏或比例不同后，原始协议角度相等未必代表纯自转，必须先核对第 6 节的标定关系。

实操步骤：

1. 启动 `python -B -m ArmDriver`。“目标与关节”页默认勾选“逆解优先电机 (3, 4) 等角/反角（软优先）”。取消即可恢复原来的候选选择。没有配置此电机对时，该选项禁用；其他通用机械臂无需配置。
2. 试位置 `[0.60,0,0.30]`，勾选“只约束位置”，求逆解并预览。求解器会先保留满足任务的原候选，再额外尝试等角/反角解，全部通过检查才优先采用。
3. 到“实机串口”页填写已确认的实际电机角度，点“计算位姿待发目标”。待发区明确显示“等角偏好已满足”“反角偏好已满足”或“保留组合解”。可以先预览待发目标，再决定下发。
4. 取消“只约束位置”，同一位置填姿态 `[15,-10,0]`。当前构型为了同时达到这些位置和方向，通常需要两电机不等角也不反角；会保留满足目标的组合解。不会把自转角清零、改成水平或拆成两次发送。

这是有效逆解之间的偏好：位置、完整姿态（启用时）、逻辑关节限位、电机限位和端点碰撞校验均保持原要求。同类候选仍按原来的关节位移和限位代价选择。搜索未找到可用对称解时正常回退，不声称证明对称解不存在。默认 IK 通过阈值仍为位置 0.005 m、姿态 0.02 rad；自写程序可用 `IKSettings` 收紧，下方示例为 1 μm 的数学检查容差，不能等同于实机精度。

**直接填写 J1～J4 并计算关节目标时不会应用这项偏好。** 例如明确要求 `[0,0,15,15]`，修正方向后得到末端 `[0,30]`，以保留用户要求。偏好针对绝对终点，不要求从任意起点出发的两电机增量也等角/反角。切换勾选会使已有待发目标失效，需重新计算；界面勾选只是本次会话开关，重新应用配置时按配置恢复。

自写程序用法：

<!-- walkthrough:preference -->
```python
from ArmController import IKSettings
from ArmDriver import ArmDriver, MotorPairPreference, load_model

model = load_model("ArmDriver/arm_config.json")
preference = MotorPairPreference((3, 4))
arm = ArmDriver(model, ik_settings=IKSettings(task_mode="position", position_tolerance=1e-6),
                motor_pair_preference=preference)
# 演示用零起点；实机须替换成已确认的实际电机绝对角度。
result = arm.solve_target_pose([0.60, 0, 0.30], current_motor_deg=model.zero_deg)
print("Motor target deg:", result.target_deg)
print("Pair relation:", preference.relation(result.motor_ids, result.target_deg))
print("Tool m:", arm.forward(result.target_deg).position)
# arm.send_target(result, sender) 仍只发送这一组绝对目标。
```

`load_model()` 只读取几何与传动，不隐式应用控制策略；自写程序需像上述代码显式传入偏好，或像第 8 节从配置读取。软优先作用于 GUI 使用的 `solve_target_pose()` 单目标接口；旧 `solve_pose()` 路径规划接口沿用原策略。此策略位于 ArmDriver，ArmController 保持通用运动学工具的职责；下位机已有 S 曲线与串口帧格式均无需更改。

## 4d. 负 X 目标：先核对工具朝向

当前 JSON 支持工具进入负 X 区域，没有 X 必须非负的限制。先在虚拟预览中复现：

1. 点击“参考姿态”，逻辑关节填 `[120,0.1,-60,0]`，点击“按关节目标预览”。工具位置应约为 `[-0.289,0,0.683146] m`，工具绝对俯仰为 60°。
2. 再点击“参考姿态”，目标 XYZ 填 `[-0.289,0,0.683146]`，Rx/Ry/Rz 填 `[0,-60,0]`，取消“只约束位置”，点击“求逆解并预览”。这组负 X 位姿可以解算，当前电机 ID 顺序下目标约为 `[120,439.02439,60,-60] deg`。
3. 同一 XYZ 勾选“只约束位置”也可以解算，但工具方向允许变化，关节解可能不同。

若姿态仍为 `[0,0,0]`，要求的是工具沿基座 +X 水平朝向。大臂角 a 在 `[0°,170°]`，末端相对俯仰 p 在 `[-90°,90°]`；保持这份方向需要 `p=-a`，所以 a 最多 90°。当前几何下 `X=(0.543+伸缩量)×cos(a)+0.065`，此时 X 不可能为负。大臂转到 120° 可以进入负 X，但继续保持原方向需要 p=-120°，超出 J3 限位。这是位置与姿态组合受到限位约束，不是逆解禁止负坐标。

你提供的目标 `XYZ=[-0.61,0,0.20]、Rx/Ry/Rz=[0,0,0]` 同样是位置可达、指定方向超限：保持 +X 朝向需要大臂约 169.099°、伸缩 0.144405 m、末端相对俯仰 −169.099°，最后一项不满足 ±90° 限位。实测解算中，勾选“只约束位置”可通过；如果希望吸盘轴改为沿 −X 水平朝向，取消该勾选并填写姿态 `[0,-180,0]`，可得到约 `[166.583816,0.017290,13.416184,0]` 的逻辑关节目标。这两种方式都通过当前模型从零位出发的理想路径检查，工具方向各不相同。

向基座 −X 的水平朝向与原来的 +X 水平朝向也不同，需要修改完整目标方向。不要为了让逆解通过而直接扩大限位，先确认机械允许的相对俯仰范围。以上例子用于虚拟验证，实机目标需结合当前起点和标定参数核对。

## 5. 接上实物：用 GUI 下发第一个小幅目标

当前提供的固件为 UART4、115200、8N1、无流控。确认板端实际引出后，使用匹配电平的 USB-TTL：TX 接 PC11/UART4_RX（当前固件；以实际 usart.c 为准），GND 接 GND。仅发送时无需连接板端 TX。确认板子运行同一版协议；RS232 电平不能直接连接 TTL。

先查看固件启动逻辑：此前 app.cpp 使能后调用 `setTarget(0)`，主机没发命令也可能运动。先确认固件零点、反馈与使能行为。调试速度/加速度/jerk 在已有下位机 ProfileConfig 中设置；Kp/Kd 用于跟踪与阻尼，不等于速度规划参数。本次没有改写或新增下位机 S 曲线算法。

具体操作：

1. 插 USB-TTL。进入“实机串口”页，点“刷新端口”，选择实际 COM（也可手动填写），点“连接”。端口被占用时先关闭其他串口助手。
2. 在下位机调试器确认四个实际电机角度、实物已经停稳，按 **`[大臂,伸缩,末端1,末端2] deg`** 输入“初始／校正用实际电机绝对角度”。读取电机反馈换算后的绝对角度，单位/正方向/零点应与协议一致；`motor_ref.position_deg` 和 `arm_rx.target_deg` 是命令目标，不是实际反馈。不要抄虚拟动画读数。
3. 假定实物确在已标定零位，且真实协议读数都是 0，才填写 `0 0 0 0`。否则使用实际读数和附近的小幅目标。
4. 到“目标与关节”页，把逻辑关节目标填成 `1,0,0,0`。返回“实机串口”页，点“计算关节待发目标”。这一步只计算，没有发送。
5. 核对待发区的逻辑关节、工具姿态、电机 ID、目标、增量和十六进制帧；可点“预览待发目标（虚拟，不发送）”检查三维结果。上述零位假设下目标/增量为 `[1,0,0,0] deg`。改变输入后需重新计算。
6. 点“下发已计算目标”。默认约每 0.05 s 重复同一绝对目标，持续 0.3 s。这是链路重同步用的重复，不是运动时长，也不是轨迹采样。
7. 界面显示“主机已完整写出 N 帧；接收/到位未确认”。观察实际动作与下位机变量。发送后输入框清空、待发目标失效。默认手动模式下，下一次重新填写实际角度；连续调试模式可沿用上次写出的目标作为估计起点，见下面操作。

连接、模型加载、预览和虚拟复位不会自动发送。位姿发送流程相同，填写 XYZ/RxRyRz 后点“计算位姿待发目标”。

**连续调试，不用每次手抄角度：**

1. 在“实机串口”页勾选“连续调试：沿用上次已发送目标作为估计起点”。
2. 首次仍需填写实际电机绝对角度。只有真实读数都是 0 时才填 `0 0 0 0`；模型零偏 `zero_deg` 是标定参数，不代表当前读数。
3. 计算并下发一次目标。主机完整写出至少一帧后，会记录线上的 float32 绝对目标；输入框留空，提示显示“上次写出的目标”。
4. 后续只需改目标位置/姿态或逻辑关节，重新点“计算位姿/关节待发目标”→“下发已计算目标”。GUI 用上次写出的目标作为本次逆解、增量和理想路径检查的估计起点。
5. 需要校正时，在输入框填写新读数：手动输入优先于历史目标。断开、短接检测、发送失败或应用模型会清除历史；重新连接后需初始化。板子复位或机构被外力移动时，主机无法自动检测，请手动重新填写实际角度。

记录的是**命令目标，不是实机反馈**，三维预览不会覆写它。尚未到位就发新目标时，起点、增量与路径检查只能基于估计；真实运动由下位机以实际状态处理。不开启连续模式时，保留原来的逐次手动填写方式。

“当前角度”不进入串口帧，也不叠加到目标上。串口始终发送绝对终点：先发 10°、再发 15°，目标就是 15°，不会累加成 25°。当前角度用于逆解种子/候选选择、增量显示和路径检查；直接关节目标的终点换算由模型决定。

**为什么大臂与两末端角度总是同样大小？** 当前零偏 0，工具绝对俯仰 `θ=大臂角 a+末端相对俯仰 p`，标定后两个末端电机为 `m1=−p+2r、m2=p+2r`。只改 XYZ 且姿态仍为 `[0,0,0]` 时，要求水平且不自转，所以 `p=−a、r=0`，得到大臂 `a`、末端1 `+a`、末端2 `−a`。旧截图的 `−a、+a` 来自修正前的俯仰列；数值同样大小源于补偿关系，与角度复制无关。改 Ry=-10° 要求向上绝对俯仰 10°，则 `p=10°−a`；再改 Rx=15°，末端目标变为 `−p+30°、p+30°`，通常就与大臂不同。也可直接输入关节目标 `[20,0.05,-10,15]`，当前配置电机目标为 `[20,219.512195,40,20]` deg。

**“停止重复下发”只取消后续重复帧，不能停止电机。** 当前协议没有 STOP；暂停动画、断开串口、关闭 GUI 都不会撤回已接收目标。实物停止需使用已有下位机保护/停止机制。

没有动作时在下位机依次检查：decode 是否通过 → target_deg[4] 是否更新 → ready 是否被取走 → motor_ref 是否更新 → setTarget 是否触发 → 轨迹使能及 CAN。连接成功或写出成功都不等于固件已接收。

## 5a. 主机写出后，Ozone 的 motor_ref 不变怎么定位

“预览待发目标”不会发送。先点“下发已计算目标”，确认出现“主机已完整写出 N 帧”。串口页现在有独立“发送记录”，后续动画提示不会覆盖这份记录；“已连接 COM”只代表打开端口。点“发送详情”可复制实际打开的端口、串口后端、115200/8N1/流控设置、目标二进制帧、完整帧数、字节数和系统发送队列字节数。它读取实际连接对象，不使用端口输入框冒充实际端口。当前发送器会有界等待系统发送队列排空，超时会报错而不显示完整成功；loop:// 的队列是待回收的回环字节，会标为“不适用”，不冒充硬件发送队列。真实串口队列为 0 仍不能证明 TX 引脚有波形或板端已接收。

**USB-TTL 不闪或板端没有接收中断时，先独立验证电脑到转换器：**

1. 关闭 GUI 和其他串口助手，避免占用串口。拔掉 USB-TTL 再刷新设备管理器，确认消失的端口确实是 COM5；插回后以实际端口为准。
2. 将 USB-TTL 与 STM32 的接线全部断开，只在转换器上短接 TX 和 RX，USB 仍接电脑。
3. 重新启动 GUI，连接刚确认的真实端口，点击“USB-TTL 短接检测（先断开机械臂）”，确认已经隔离板端并短接。它使用 GUI **当前实际打开的同一个串口对象**发送 1600 字节测试数据并读取回环。显示 `COM5 USB-TTL 短接回环 PASS；写入 1600 / 回收 1600 字节` 才是本体回环通过。`loop:// 软件回环 PASS` 不算硬件验证。
4. 若要独立于 GUI 复核，先关闭 GUI，再在 Python 中执行下面代码，port 改成刚确认的真实端口。它发送的是 ASCII 测试数据，测试序列不含 A5，也不是机械臂角度帧。接线必须按上一步隔离后再运行。

<!-- walkthrough:ttl_loopback -->
```python
import serial

port = "COM5"  # 必须是转换器真实端口；loop:// 仅为软件回环，不能检验硬件
payload = b"ArmDriver UART loopback\r\n" * 64
with serial.serial_for_url(port, baudrate=115200, bytesize=8, parity="N", stopbits=1,
                           timeout=2, write_timeout=1, xonxoff=False,
                           rtscts=False, dsrdtr=False) as link:
    link.reset_input_buffer()
    written = link.write(payload)
    received = link.read(len(payload))
    print("Actual port:", link.port)
    print("Written:", written, "Received:", len(received))
    print("PASS" if written == len(payload) and received == payload else "FAIL")
```

短接正确且 PASS，证明这台电脑经该端口的转换器 TX/RX 回路可用；若 FAIL，只能确定测试链路未通过，继续检查端口身份、短接、驱动和转换器，不能先归罪于 STM32。LED 是否闪烁不是最终判据，少量数据可能很难看清。PASS 后恢复板端接线、重新打开 GUI，再核对“发送详情”的实际端口及帧；GUI 的短接检测已使用同一连接对象验证收发；正常机械臂控制流程仍不读取下位机回传。检测后需要重新确认实机角度、重新计算待发目标，不能沿用检测前的待发指令。

短接 PASS、恢复板端接线后仍无 UART4 中断时，先检查运行时配置，而不是仅看 CubeMX 设置。当前固件已改为 PC11 RX / PC10 TX：在 Ozone Watch 查看 `(GPIOC->MODER >> 22) & 3`（应为 2，PC11 复用模式）、`(GPIOC->AFR[1] >> 12) & 15`（应为 8，AF8_UART4）、`(RCC->APB1ENR >> 19) & 1`（应为 1，UART4 时钟开启）。恢复接线前移除转换器 TX/RX 短接；仅发送时可以只接转换器 TX→板端 PC11 与共地。这三项均正确，且 SR 仍无 RXNE 时，再用逻辑分析仪/示波器比较转换器 TX 和板端 PC11 上的波形，排查接线通路、电平或输入负载；短接通过不证明数据已到 MCU 引脚。

当前源码是 UART4 RX=PC11、TX=PC10，115200/8N1。USB-TTL 的 TX 接 PC11，GND 必须接板子 GND；仅发送不要求接板端 TX。COM5 的 CH340 被枚举只能证明电脑识别了转换器，不能证明它接到了这个 UART。

在当前 Debug 固件的 Ozone 中展开 `arm_rx`，再次下发**已经确认可用的小幅目标**，比较前后的计数。不要只看很快被任务清零的 `ready`：

| 观察值 | 含义与下一步 |
| --- | --- |
| `hdr_match_cnt` 始终 0 | 没找到 A5；核对共地、接线、实际引脚、波特率及 UART4_IRQHandler 是否执行 |
| `hdr_match_cnt` 增长，`data_received_cnt` 不增长 | 帧头进入了，但后续 DMA 未完成；看 DMA1_Stream2_IRQHandler、huart4.ErrorCode、hdma_uart4_rx.ErrorCode |
| `data_received_cnt` 增长，`decode_fail_cnt` 增长 | 完整数据进入，但 CRC/数值校验失败；看 arm_rx.rx_buffer_ 的 19 字节，和 GUI 的帧逐字节核对，不可发送十六进制文本 |
| `decode_success_cnt` 增长，`target_deg` 正确 | 串口已成功解码；再查 TIM_Callback_1kHz 和 MotorTask 的 check_target 分支是否执行，以及 arm_rx.isConnected() 是否为 true |
| `motor_ref` 已更新但电机不动 | 通信已打通；再查 last_target、traj.setTarget、MIT/CAN 和电机使能 |

`arm_rx.target_deg` 和 `motor_ref` 数组顺序均是 **[末端1,末端2,大臂,伸缩]**，不是 GUI 模型顺序。例如关节 `[17.277901,0.130386,-17.277901,0]` 在方向修正后对应固件数组约 `[+17.277901,-17.277901,17.277901,572.426331]`，旧截图中的末端两值符号相反。数值以本次待发的 float32 为准，不能拿 J3/J4 直接对应 motor_ref[0]/[1]。

计数器仅在定义 `DEBUG` 的固件中存在。如果 Ozone 无法展开，确认正在运行的 ELF 与重新编译下载的固件一致。动态 Watch 采样看到 ready=false 很常见；它不是“没有接收”的证据。避免长时间停在接收 ISR 中单步，否则会改变 DMA 与 watchdog 的时序。当前软件没有下位机回传，上位机无法仅凭“写出”判断是哪一步阻塞。

如果所有计数都为 0 且 `state_=WaitHead`，说明还未找到帧头。先看发送前后的 `arm_rx.rx_buffer_[0]`（收到的单字节）、`huart4.RxState`、`huart4.RxXferCount` 和 `huart4.ErrorCode`。`WaitHead` 使用 IT 接收，接收中应是 BUSY_RX；重装下一字节后 RxXferCount 又变为 1，单看这个数不证明没收到。在 `Core/Src/stm32f4xx_it.c` 的 `UART4_IRQHandler` 内，给 `HAL_UART_IRQHandler(&huart4);` 设断点，运行后下发，观察是否命中：若不命中，先查接线/实际 RX 引脚 和 IRQ；若命中但没进入 `ArmUartRx` 的接收回调，查 ErrorCode、RXNE/RXNEIE 状态及接收完成回调注册；若回调执行而没有 A5，检查电平/波特率和收到的字节。不要因这些计数为零就去改 IK 或 CRC。

若已看到 `rx_buffer_[0]=165`（A5）、`hdr_match_cnt` 增长，但 `data_received_cnt=0` 且停在 `Receiving`，应查帧头后的 DMA，不再查 IK。观察 `huart4.hdmarx->State`、`ErrorCode`、`Instance->NDTR`、`Instance->CR` 与 `huart4.Instance->CR3`：成功启动循环接收时 UART 应为 BUSY_RX，DMA CR.EN=1、CR3.DMAR=1，NDTR 随字节搬运递减；不要用 UART 的 RxXferCount 代替 DMA NDTR。

接收库现已检查 IT/DMA 启动返回值。DMA 启动失败会退回搜索帧头，避免状态假装 Receiving；Debug 的 `last_dma_start_status`（0=OK、1=ERROR、2=BUSY、3=TIMEOUT）和 `dma_start_fail_cnt` 可观察启动失败。`last_dma_start_uart_error`、`last_dma_start_dma_error`、`last_dma_start_dma_state` 在回退清除错误前保留失败现场。若 IT 也启动失败则为 Stopped，查看 `last_it_start_status` / `it_start_fail_cnt`。这修复了状态处理，但具体 DMA 硬件原因仍需板端值确认。

修改后须重新编译、下载，并让 Ozone 加载匹配的新 ELF；仅重启上位机 GUI 不会更新 MCU 程序。不要保留旧接收 ISR 断点单步接收整帧，这会改变后续字节和 DMA 的时序。

## 6. 标定方向与末端差动

分别从已确认的参考姿态做小动作，每次重新核对实际角度：大臂 `1,0,0,0`；伸长 1 mm `0,0.001,0,0`；俯仰 `0,0,1,0`；吸盘自转 `0,0,0,1`。示例电机增量分别为 `[1,0,0,0]`、`[0,4.390244,0,0]`、`[0,0,-1,1]`、`[0,0,2,2]`。

方向反了先核对固件 reverse、auto_zero、reduction_rate 与角度域，再改 JSON 的 direction/C 和零偏。不要同时改多个方向定义。

保持其他关节不变，测一组已知俯仰 p 和一组吸盘自转 r。C 的末端俯仰列填 `[Δend1/p,Δend2/p]`，自转列填 `[Δend1/r,Δend2/r]`，先扣零偏。若列 gear_ratio/direction 不为 1，需除以该比例，避免重复计入。只测俯仰无法确定自转列。保存后先断开串口，重新应用，再做预览与小动作验证。

多轴协同仍需由下位机接收整组目标后，按物理电机位移分配已有规划器的 v/a/j，再统一启动。当前角度帧没有 v/a/j、同步时长或共同进度字段；主机同时发送四个目标并不能让独立 S 曲线天然同步到达。ArmDriver/stm32 已删除，无需保留另一套曲线实现。

## 7. 自己写程序：直接加载同一份 JSON

不必另写 Python 建模文件，也不必导入 GUI。下面代码在项目根目录执行，检查模型、传动和 FK：

<!-- walkthrough:model -->
```python
from ArmDriver import load_model, ArmDriver

model = load_model("ArmDriver/arm_config.json")
arm = ArmDriver(model)
q = model.user_to_joints([20, 0.05, -10, 15])  # 旋转 deg，伸缩 m
print("Motor deg:", model.joints_to_motor(q))  # 当前零偏/传动下 [20,219.512195,40,20]
print("Tool m:", model.kinematics.forward(q).position)
```

程序导出模型使用模型层接口；GUI 与自写程序读取同一格式：

<!-- walkthrough:export -->
```python
import json
from pathlib import Path
from ArmDriver import model_to_dict

configuration = json.loads(Path("ArmDriver/arm_config.json").read_text(encoding="utf-8"))
configuration["model"] = model_to_dict(model)
Path("ArmDriver/my_arm.json").write_text(
    json.dumps(configuration, ensure_ascii=False, indent=2), encoding="utf-8")
```

## 8. 最简完整控制代码：预览 → 回环 → 实机

这是自定义程序参考；日常调试使用 GUI 即可。新建 `ArmDriver/my_control.py` 并粘贴。六个 --pose 参数为 X/Y/Z（m）、Rx/Ry/Rz（deg），与 GUI 一致。

<!-- walkthrough:control -->
```python
import argparse
import json
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scipy.spatial.transform import Rotation
from ArmDriver import ArmDriver, MotorPairPreference, load_model, UartRxSyncSender

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("arm_config.json"))
    connection = parser.add_mutually_exclusive_group()
    connection.add_argument("--port")
    connection.add_argument("--loopback", action="store_true")
    parser.add_argument("--current-motor-deg", type=float, nargs="+")
    goal = parser.add_mutually_exclusive_group()
    goal.add_argument("--joints", type=float, nargs="+")
    goal.add_argument("--pose", type=float, nargs=6)
    args = parser.parse_args()
    if args.port and args.current_motor_deg is None:
        parser.error("实机必须填写 --current-motor-deg 已确认的实际角度")
    model = load_model(args.config)
    configuration = json.loads(args.config.read_text(encoding="utf-8"))
    uart = configuration["uart"]
    options = configuration.get("control", {}).get("motor_pair_preference")
    preference = MotorPairPreference(**options) if options is not None else None
    arm = ArmDriver(model, motor_pair_preference=preference)
    # 没有实物时才假定从模型零位开始。
    current = args.current_motor_deg if args.current_motor_deg is not None else model.zero_deg
    if args.pose:
        quaternion = Rotation.from_euler("xyz", args.pose[3:], degrees=True).as_quat()
        target = arm.solve_target_pose(args.pose[:3], quaternion, current_motor_deg=current)
    else:
        target = arm.solve_target_joints(args.joints or [20,.05,-10,15], current_motor_deg=current)
    encoder = UartRxSyncSender(None, channel_motor_ids=uart["channel_motor_ids"])
    angles = encoder.quantize_angles(target.motor_ids, target.target_deg)
    model.collision.require_valid(model.motor_to_joints(angles))
    frame = encoder.encode_angles(target.motor_ids, angles)
    print("Target motor deg:", target.target_deg)
    print("Delta motor deg:", target.delta_deg)
    print("Frame:", frame.hex(" "))
    if not (args.port or args.loopback):
        print("Preview only")
        return
    with UartRxSyncSender.open("loop://" if args.loopback else args.port, **uart) as sender:
        count = arm.send_target(target, sender, repeat_for_s=0 if args.loopback else .3)
        if args.loopback:
            sender.stream.timeout = .5
            assert sender.stream.read(len(frame)) == frame
            print("Loopback OK")
        else:
            print("Written frames:", count, "; reception/arrival unconfirmed")

if __name__ == "__main__":
    main()
```

先执行以下三条，不连接实物：

```powershell
python -B ArmDriver/my_control.py --joints 20 0.05 -10 15
python -B ArmDriver/my_control.py --loopback --joints 20 0.05 -10 15
python -B ArmDriver/my_control.py --loopback --pose 0.60 0 0.30 15 -10 0
```

关节示例得到 `[20,219.512195,40,20]`，帧角度通道为 `[40,20,20,219.512195]`。两个回环命令应打印 `Loopback OK`，只证明字节链路正确，不模拟机械运动。

完成前面实机小幅验证后，自定义程序的实机命令如下。端口、起点和目标必须换成实测值；下面仍假定已确认四个实际电机均为零：

```powershell
python -B ArmDriver/my_control.py --port COM5 --current-motor-deg 0 0 0 0 --joints 1 0 0 0
```

一次只发送一个整组绝对目标。固件只保留最新目标，没有路点队列；等实际停稳、核对位置后再发下一条。此代码不做速度规划，基本流程为 `load_model → ArmDriver → solve_target_* → send_target`。

## 9. 协议、故障处理和软件验收

当前帧固定 19 字节：`A5 + 四个小端 float32 deg + 小端 CRC16`。没有换行、JSON、ID 或速度字段。CRC 初值 FFFF、多项式 A001，只覆盖 16 字节角度；`123456789` 的 CRC 为 4B37。串口层校验数值、映射、float32、短写与超时；驱动再次检查量化后的电机限位和端点碰撞。部分写入失败后需重新连接、确认实物状态并重新计算，不能直接续发。

| 现象 | 实际处理 |
| --- | --- |
| No module named ArmController | 使用正确环境，在项目根目录运行 `python -m ArmDriver`；GUI 也支持直接运行文件 |
| 逆解失败 | 检查 XYZ 单位、Y=0、目标方向、尺寸和行程，先试第 4 节可达目标 |
| motor bounds | 查看合成后的物理电机角度、零偏和伸缩每转位移，两种限位都要满足 |
| collision / 余量不足 | 核对包络端点、半径、零位与误差预算；不要排除碰撞部件来绕过检查 |
| 无法打开 COM | 刷新端口、检查驱动，关闭占用该 COM 的程序 |
| “写出”但实物不动 | 按第 5 节检查 decode、目标快照、轨迹使能和 CAN |
| 目标改了不能直接发送 | 点“计算关节/位姿待发目标”再下发；手动模式需填写实际角度，连续模式可沿用上次目标估计 |
| 改 JSON 后没有变化 | 外部编辑需重新打开；GUI 内编辑需应用，保存才会写入文件 |

以下回归与性能命令用于包含 tests 和 tools 的完整开发工作区；本仓库只发布 ArmDriver 和 ArmController，不包含这些辅助文件。

回归与性能检查（完整开发工作区）：

```powershell
python -B -m unittest tests.test_readme_workflow tests.test_debug_gui tests.test_uart_sync tests.test_motor_preference
python -B -m unittest discover -s tests
python -B tools/benchmark_debug_gui.py
```

文档回归提取执行本文模型/导出/吸盘自转/软优先/控制代码，运行关节与位姿的 pyserial 回环。GUI 回归实际操作 Tk，检查预览不发数据、实际起点必须填写、输入修改使待发目标失效、发送/取消/故障/关闭清理及通道重排。绘图继续复用三维图元和背景，仅局部刷新；串口连接、发现端口和发送在后台执行，避免阻塞动画。

这些属于软件验证；真实接收、方向、差动比例、碰撞包络和到位误差仍需按第 5～6 节在实体上验证。单向协议下 GUI 始终不会显示“实机已到位”。

本次已执行文档代码与实际参数的 GUI 位姿预览/串口回环；此前完整回归 122 项通过；串口诊断更新后的 53 项专项回归及新增“写入成功但无回收必须 FAIL”测试均通过。当前电脑局部绘图刷新中位耗时约 7.5 ms，完整 Tk 帧约 14.8 ms，两秒动画显示 102 帧。当前可枚举到 CH340 的 COM5；未通过本次软件测试向实机发送目标或验证实机到位。用户已确认真实 USB-TTL 短接测试通过；恢复板端接线后的 WaitHead / 全零接收计数仍需按第 5a 节排查。
