# 导出校验记录

日期：2026-10-22。源：原仓库 `source/loco_mani_demo/loco_mani_demo/assets/go2_piper/usd_d435/` 的当前 v4 资产。准确文件哈希见 `asset_manifest.json` 和 `SHA256SUMS`。本记录适用于该次导出；修改模型或重新生成后，请重新执行校验。

已执行：

- 全部交付文件 SHA-256 核对，以及 MJCF、URDF、SDF 的网格、include 和 URI 引用闭包检查。运行资源没有仓库外绝对路径，也没有软链接或缺失 MTL。
- MuJoCo 3.9.0 编译机器人及带地面场景：25 个刚体、20 个活动关节、`nq=27`、`nv=26`、19 个执行器、2 台相机。home 初态完成 50 个物理步，状态有限。
- 两份 URDF 及 SDF 的 9 组关节姿态 FK，与 MuJoCo 逐刚体比较，最大齐次矩阵元素误差约 `1.34e-15`。核对全部刚体质量、质心及旋转后的惯量矩阵。
- ROS Jazzy 的原生 `check_urdf` 分别解析相对路径版及 `package://` 版 URDF，结果通过；浮动基座为 `base`，所有腿部及机械臂关节保留。
- OpenUSD 打开原样复制的 USD：25 个刚体、20 个活动关节、46 个碰撞体；米制、Z 向上；`ComputeAllDependencies` 未发现外部资产或未解析资源。
- 便携 MJCF 与 USD 的全部刚体初始坐标、质量及两台相机的 ROS 光学坐标交叉检查通过；刚体变换矩阵最大元素误差约 `6.21e-8`（USD 单精度旋转），两种格式均为 46 个碰撞体。
- MuJoCo EGL 离屏渲染通过；本包 `preview.png` 为该模型的装配预览。
- 压缩包解压到仓库外临时目录，从另一个工作目录运行完整校验，检查摆脱原路径后的可用性。

当前机器未安装 Gazebo Sim / `gz` / SDFormat 原生解析器，**没有执行 Gazebo 启动、渲染或动力学验收**。SDF 已检查 XML、资源引用、关节图、正向运动学、质量与惯量。预览 world 面向 Gazebo Sim Harmonic；接收者应在自己的安装上运行：

```bash
gz sdf -k gazebo/model.sdf
export GZ_SIM_RESOURCE_PATH="$(dirname "$PWD")${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim gazebo/preview.sdf
```

本次未重新运行 Isaac Sim GUI、ROS 2 colcon 构建或站立/抓取策略。USD 与原资产逐字节一致；跨仿真器的动力学、相机内参和控制效果需要分别接入与验证。
