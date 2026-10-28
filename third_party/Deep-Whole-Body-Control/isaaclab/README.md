# Go2_Piper_Lunar_Tracking：Isaac Sim 5.1.0 迁移

此目录是 Deep-Whole-Body-Control 在本工程中的 Isaac Sim / Isaac Lab 后端。新任务名为 **Go2_Piper_Lunar_Tracking**，默认从仓库的 **Lunar** 场景裁剪地形，学习机身移动与末端执行器位置轨迹跟踪。

保留并直接调用本仓库 `rsl_rl/rsl_rl` 的双输出头 ActorCritic、双价值 PPO、优势混合与历史编码器。环境改为 Isaac Lab DirectRLEnv，替换 Gym 的仿真创建、刚体/关节张量、接触、重置、相机和步进接口。原 WidowGo1/Isaac Gym 任务仅保留作参考，没有宣称把所有历史任务移植完毕。

这属于论文 Deep WBC 所使用的统一学习策略，动作经过有力矩限制的关节 PD 执行；不是基于动力学约束求解 QP 的传统 WBC。当前任务不训练抓取、夹持载荷上限或视觉感知。

## 启动

从 **Go2Piper_Lunar 仓库根目录** 执行：

```bash
# 已有环境：Isaac Sim 5.1.0 + Isaac Lab 2.3.2
export ISAAC_SIM_PYTHON="$HOME/anaconda3/envs/isaaclab/bin/python"

# 正式训练，默认任务名和场景已经配置好
bash scripts/train.sh --headless --num-envs 64 --iterations 10000

# 明确指定任务
bash scripts/train.sh --task Go2_Piper_Lunar_Tracking --headless --num-envs 64

# 短程自检
bash scripts/train.sh --mode validate --headless --num-envs 2 --steps 120

# 小规模训练检查；这不会产生已收敛策略
bash scripts/train.sh --headless --num-envs 4 --iterations 22 \
  --steps-per-env 32 --epochs 2 --episode-seconds 4 \
  --log-dir outputs/my_smoke

# 恢复模型、PPO/历史编码器优化器及训练轮数；恢复时开始新回合
bash scripts/train.sh --headless --checkpoint outputs/my_smoke/model_22.pt \
  --iterations 100 --log-dir outputs/my_resume

# GUI 回放；绿色是目标，红色是虚拟 TCP；默认使用历史编码器
bash scripts/play.sh --checkpoint outputs/my_resume/model_122.pt --num-envs 1 --steps 1200

# 按最终指令范围评估；--teacher 可切换到特权信息教师策略
bash scripts/play.sh --headless --checkpoint outputs/my_resume/model_122.pt \
  --num-envs 1 --steps 1200 --full-curriculum

# 离屏回放并保存最后一帧
bash scripts/play.sh --headless --enable_cameras --capture \
  --checkpoint outputs/my_smoke/model_22.pt --num-envs 1 --steps 120
```

原入口 `legged_gym/legged_gym/scripts/train.py` 和 `play.py` 也转发到此后端。原脚本原文保存在相邻的 `legacy_isaacgym/`，新训练入口不导入 `isaacgym`，也不要求安装旧版 Gym。旧 `save_jit.py` 仍属于历史 Gym 工具，不是新模型的导出接口。

使用本机已安装的 Isaac Lab，不修改其全局源码；启动器优先加载本克隆仓库内的旧版 DWBC PPO，避免与全局 `rsl-rl-lib 3.x` 混用。**不要为此运行旧仓库的 `pip install -e rsl_rl` 或旧 Gym 安装步骤。** 不需要 W&B 登录；只写本地 JSONL、TensorBoard 和检查点。运行时版本、源码 SHA256、资产来源与完整参数写入 `run.json`。

## 参数与场景

主配置：[`configs/training/Go2_Piper_Lunar_Tracking.json`](../../../configs/training/Go2_Piper_Lunar_Tracking.json)。

物理参数继续读取根目录 `env/config/Lunar.json` 或 `Lunar_baseline.json`，关节限位/力矩上限读取 `assets/go2_piper_d435_assets/joint_metadata.json`。PD 增益读取场景配置，避免使用源资产里不同的 Isaac Lab 示例增益和力矩上限。

```bash
bash scripts/train.sh --headless --env Lunar_baseline --num-envs 64
bash scripts/train.sh --headless --env Lunar --num-envs 64 \
  --slope-deg 5 --foot-static-friction 0.6 --foot-dynamic-friction 0.5 --mass-kg 5
```

- 默认重力 1.62 m/s²，物理步长 1/240 s，策略降采样 4，即 60 Hz。
- 默认足地静/动摩擦 0.8/0.65；这些是可配置实验假设，不是已标定的月壤常数。地面系数 1 与 multiply 合并模式保留。
- 默认从当前场景中央裁剪约 16 m × 16 m。裁剪沿原网格边界，实际 Lunar 范围约 ±7.904 m；顶点和三角形保持不变，保留区域内散石和碎石。每个并行环境重复同一块地形；不等于不同月面区域的数据泛化。
- 不把 3 km 远景复制到每个训练环境。原完整展示场景仍通过 `bash env/run.sh --env Lunar` 打开。
- 裁剪场景、可达目标点库、物理参数清单缓存在 `assets/lunar_training/<hash>/`；地形仍来自项目中已有 DEM/基线资产，无网络下载依赖。
- GPU PhysX 不支持 CCD，本后端 GPU 模式显式关闭 CCD，采用 240 Hz 离散接触；CPU 模式开启。展示场景原有 CPU CCD 设置不改动。
- 矿石仍是动态刚体，质量选项只改变矿石本身，不代表给机械臂附加载荷。该任务没有矿石抓取奖励或刚性附着。
- D435 的安装几何保留；训练观测使用仿真真值，不采集双 RGB-D 图像，不是视觉策略。

恢复检查点时默认加载其保存的配置；若控制含义、物理环境或关节顺序不一致，会明确拒绝。批量大小可以改变。训练缓存失效会重新生成，默认展示场景不被覆盖。

## 任务定义

动作固定顺序：FL、FR、RL、RR，每条腿 hip/thigh/calf，然后 Piper joint1…joint6。18 维归一化动作裁剪到 [-1,1]，乘各关节 action_scale 并加初始关节角，再限制在关节范围内。两个夹爪关节保持当前配置开度；默认 joint7=0.03 m、joint8=-0.03 m。整个机器人只有这一处动作写入，不与独立站立控制循环竞争。

移动指令为水平航向坐标系中的 vx、vy 与世界 Z 轴角速度。默认完整范围 vx=[-0.2,0.5] m/s、vy=[-0.2,0.2] m/s、转向=[-0.5,0.5] rad/s，每 5 s 重采样，20% 为站立指令。

末端定义为 `link6` 坐标系下 [0,0,0.18] m 的虚拟 TCP。这个偏置是可配置的任务定义，需要按实物夹爪标定。目标点在本机器人的真实 USD 关节链上采样 FK 得到，再进行空间范围筛选；这不是完整的无碰撞 IK 规划器。目标之间采用五次平滑曲线，运动 2–4 s，保持 0.5–1.5 s。

轨迹坐标系仅随基座航向旋转，原点 XY 随机器人移动，Z 为当前地形高度 + 0.34 m。目标不随机身 roll/pitch 或实际机身高度一起倾斜，允许学习身体与机械臂的协同补偿。该任务默认是**相对移动基座的末端轨迹**，不是世界固定位置的抓取目标。

课程从最终速度与末端目标偏移的 20% 开始，在 2000 个更新轮次内线性增加到 100%。当前是按轮数推进的课程，不是按成功率自适应推进。

奖励分为移动和机械臂两路，保留原 PPO 的双价值学习与优势混合：水平速度、转向跟踪、存活、姿态/高度约束、脚部滑动、异常接触、关节功率、动作变化等进入移动奖励；TCP 位置/速度跟踪、机械臂功率和动作变化进入机械臂奖励。全部按策略时间步缩放。两路混合系数随训练推进。

观测一帧 83 维：基座线/角速度、重力投影、移动指令、18 个关节位置偏差和速度、上一动作、四足接触、TCP 位置/目标/误差/目标速度、离地高度。另有 6 维特权环境参数和 10 帧历史，共 **919 维**。历史回放时不使用特权编码器。当前特权参数在单次训练中固定，尚未移植原论文完整的动力学域随机化；不能据此声称具备 sim-to-real 鲁棒性。

终止条件包括倾倒、机身过低/过高、基座碰撞、越出裁剪边界和无效数值；超时与失败分开。超时使用**自动重置之前的终止状态价值**进行 bootstrap，局部重置同时清除动作、历史、轨迹时钟并恢复矿石。

## 文件职责

| 路径 | 职责 |
| --- | --- |
| `dwbc_isaaclab/config.py` | 配置校验、关节顺序、观测协议 |
| `assets.py` | 从现有场景裁剪地形、设置机器人、生成 FK 目标点库和来源清单 |
| `env_cfg.py` | Isaac Lab 场景、物理、隐式关节驱动与接触传感器配置 |
| `tracking_env.py` | 批量状态、移动/轨迹指令、动作、观测、奖励、终止和重置 |
| `math_utils.py` | 四元数、五次轨迹、与地面三角形一致的高度查询 |
| `runner.py` | 调用原双头 PPO、历史更新、保存/恢复、离线日志 |
| `validation.py` | 真实仿真、接触参数、重力、地形来源、重置与回放检查 |
| `launch.py` | 启动 SimulationApp，注册并创建任务，选择 train/play/validate/prepare |
| `tests/test_core.py` | 无需启动仿真的数学、协议和 PPO 回归检查 |

训练输出位于 `outputs/Go2_Piper_Lunar_Tracking/<时间戳>/`：`run.json`、`metrics.jsonl`、`tensorboard/`、`model_N.pt`、`training_result.json`。回放可另存 `play.json` 与 `scene.png`。日志中的短程误差只用于调试，不是正式性能评估。

```bash
"$ISAAC_SIM_PYTHON" -m unittest discover -s third_party/Deep-Whole-Body-Control/isaaclab/tests -v
"$ISAAC_SIM_PYTHON" -m unittest discover -s env/tests -v
"$HOME/anaconda3/envs/isaaclab/bin/tensorboard" --logdir outputs
```

## 迁移与引用

源仓库基于 commit `8159e4ed8695b2d3f62a40d2ab8d88205ac5021a`。源代码版权、许可证与论文引用保持原样。旧 PPO 的本地修改包括终止状态 bootstrap 和 PyTorch 2.x 分布标准差保护；本后端不用原先依赖 W&B 的 runner。

- Fu, Cheng, Pathak. [Deep Whole-Body Control: Learning a Unified Policy for Manipulation and Locomotion](https://arxiv.org/abs/2210.10044), CoRL 2022。
- [Isaac Lab 的 Isaac Gym 迁移说明](https://isaac-sim.github.io/IsaacLab/v2.3.0/source/migration/migrating_from_isaacgymenvs.html)。
- [Isaac Lab DirectRLEnv 任务接口](https://isaac-sim.github.io/IsaacLab/v2.3.0/source/tutorials/03_envs/create_direct_rl_env.html)。
- 月面数据和合成细节的说明见项目 `assets/terrain_data/apollo17/README.md`。

本次交付验证的是训练链路可运行。正式研究仍需充分训练，按独立随机种子、最终指令范围、不同地形/摩擦条件统计移动误差、末端误差、跌倒率、关节饱和和成功回合比例；之后再扩展接触抓取和载荷课程。
