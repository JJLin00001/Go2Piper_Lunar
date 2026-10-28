# Model provenance — 2026-09-23 (STEP CAD nominal)

## Go2、Piper 与活动模型

Go2、Piper、夹爪及历史 DaBai 资产来自
`/home/jjlin/go2-piper-mobile-manipulation-src/assets/mujoco`。
`reference/MUJOCO_SHA256SUMS` 记录保留的源模型树及其项目文档快照，
**不包含**本项目的 D435 overlay；后者由
`reference/D435_SHA256SUMS` 记录。历史 DaBai XML/网格保留用于复现，
不由活动场景别名或 ROS 生成器选用。

活动组合模型为 `mujoco/combined/go2_piper_d435.xml`，另外两份活动
PiPER 模型位于 `mujoco/piper_d435/`。ROS Xacro 由
`scripts/generate_go2piper_description.py` 从活动组合模型生成；
`--check` 用于检查生成结果是否漂移。

## 用户连接件 STEP 与名义装配

用户提供实物连接件模型
`reference/operator_mount/Piper夹爪相机支架.step`，其 STEP 单位为毫米，
SHA-256 为 `57dd501344998fb155413c9a4b29b341a8d814e90dafd2c9a94a6928416ead9b`。
活动模型使用由该文件生成、转换为米制的
`mujoco/piper_d435/assets/piper_d435_wrist_mount.stl`，SHA-256 为
`ce6097392dd201a2d930a1772fc8799b675abdef460030c3e115f1fa28ccb439`。
历史 `piper_dabai/assets/realsense_mid_stand_v2.STL` 没有被覆盖。
STEP 的制造版本、材料、实物尺寸公差及其与已安装件的逐项一致性为
**UNKNOWN — requires verification**。

用户提供的四张装配照片位于 `/home/jjlin/图片/前.jpg`、`上.jpg`、
`后.jpg`、`左.jpg`；用户明确指出相机朝前，后背两颗螺丝固定在远离
机械臂的一排。当前 CAD 名义装配据此采用 STEP 相机端远端 B 排，
并依据照片推定朝支架内侧的接触面；该面尚未经实测确认。STEP 机械臂端 12 mm 双孔配准到现有
`gripper_base_100_v3` 双孔。相机端 45 mm 双孔再与 D435 DAE 的后背
M3 双孔配准。这是几何/照片约束下的**名义姿态**，并非实测装配公差、
螺钉咬合、线缆走向或安全碰撞包络的验收。四张照片仍在仓库外，
其后续可复现性取决于该本地路径。

## 官方 D435 模型与坐标约定

D435 可视网格、参考 Xacro 和许可证来自官方
`realsenseai/realsense-ros` 固定提交
`60c850958d651130fc2cc3d10efb37ff5be93da5`（项目使用的 ROS 包版本
4.58.3）：`meshes/realsense/d435.dae`、
`reference/realsense_d435/_d435.urdf.xacro` 和
`reference/realsense_d435/LICENSE`。官方 DAE 声明米制、`Z_UP`；
其原样资源与项目 D435 overlay 均由 `reference/D435_SHA256SUMS` 约束。
用户提供的 STEP **不是**官方 RealSense 资源。

活动 `link6_T_camera_link`（`camera_link` 即模型中的
`d435_camera_body`）来自上述 STEP B 排内侧面装配，而不再从历史手眼
结果反推。生成器当前写入：

- xyz（m）：`[-0.073566718231561, 0.028486264512332, 0.039196734972023]`
- RPY（rad）：`[-0.001692832098986, -1.220934030738766, -0.000794180072340]`

URDF 层级为
`link6 → wrist_camera_mount → d435_bottom_screw_frame → d435_camera_body`。
`link6_to_wrist_camera_mount` 为 identity；其网格本身携带 link6 下的 CAD
姿态。`d435_bottom_screw_frame` 沿用官方 Xacro 的**底部单孔坐标基准**，
`bottom_screw_frame_T_camera_link=(0.0106, 0.0175, 0.0125) m`、零 RPY；
它不表示实际使用底部单孔固定。真实固定约束是后背 B 排两个 M3 孔。
活动 MuJoCo 仍将支架、D435 box 和名义模拟相机直接置于 link6，
ROS 固定关节仅将同一几何表示为独立 link。

官方 D435 碰撞 primitive 为 `0.02505 × 0.090 × 0.025 m` 盒；
MuJoCo 不能解码上游 DAE，因此活动 MJCF 的相机 visual/collision 均
使用该盒，URDF/RViz visual 使用 DAE。当前盒在后侧约有 `8.23 mm`
不能覆盖 DAE 可视几何；更不用说支架螺钉、突出物和自由线缆。
因此它是当前软件碰撞近似，**不是**已批准的实物相机包络。
ROS 规划把 STEP 派生支架网格保守复用为 `wrist_camera_mount` 碰撞体；
MuJoCo 中该网格仍是 visual，不产生接触。

## 历史手眼结果与未解一致性

2026-08-27 的 `link6_T_camera_color_optical_frame` PARK 手眼记录及同序列号
D435 的 factory extrinsics 仍保存在 `config/calibration.yaml` 和历史证据中。
它们曾导出旧 `link6_T_camera_link`：xyz
`[-0.0610039815132391, 0.0125814268551635, 0.00525281034929561] m`、
RPY `[-0.0621368480767478, -1.24594865022645, 0.164607156810976] rad`。
历史记录属于旧软件模型来源，不能写作当前名义装配位姿。

按当前 CAD 名义装配与原归档 factory extrinsics 推得的光学姿态，
与保留的手眼数值相差约 `39.36 mm / 6.47°`。原因尚为
**UNKNOWN — requires verification**，需独立核对当时 `link6` 基准、标定板
尺寸/目标、实物安装和手眼结果，不能凭 CAD 差异直接改写标定数值。
`config/calibration.yaml` 已将 `hand_eye_validated` 置为 `false`，
原始手眼数据未删改；URDF 不另建 `camera_color_optical_frame`，避免第二
光学 TF authority。现有手眼 TF 若被启动，其数值与当前 CAD 名义
相机本体**不能视为一致**；运动和基座 XYZ 验收须保持关闭。

## 保留的接口和安全边界

- `base` 在 ROS 中称为 `base_link`；Go2 腿在源模型 home 姿态固定。
- `piper_base_link` 默认在 Go2 基座上方 70 mm，Joint1 再高 123 mm。
- 固件 DH 选择、控制器关节名、硬件限制、夹爪轴向及
  `grasp_center` Z=0.11003 m 保持原有接口。
- URDF 将源 capsule 碰撞近似展开成 cylinder 加端部 sphere。
- SRDF 不恢复旧的非相邻 `Never` 碰撞豁免；相机和支架对 link5、
  夹爪手指的碰撞检查仍保留。
- link6 质量/惯量仍是旧载荷复合估计，不是 D435/STEP 支架的实测动力学。

实物支架、螺钉、自由线缆、完整 TF、碰撞包络和动力学均尚未验收。
模型哈希与离线软件检查均不代表实机运动许可；旧模型的规划结果不可
直接迁移为当前模型的通过证据。
