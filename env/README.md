# Lunar 与 Lunar_baseline 使用说明

## 环境与数据

| 环境 | 地形 | 默认场景 |
| --- | --- | --- |
| `Lunar_baseline` | 原有程序化起伏、24 个合成陨石坑、100 个散石；半径 1.5 m 的平面作业区 | [Lunar_baseline.usda](generated/Lunar_baseline.usda) |
| `Lunar` | Apollo 17 的 1.5 m 网格实测 DEM；保留天然地形，补充合成材质、100 个散石和 1800 个带碰撞的细碎石 | [Lunar.usda](generated/Lunar.usda) |

`Lunar_baseline` 的地面顶点与原场景逐点相同。两个环境共享用户提供的 Go2 + Piper + D435 资产、浮动基座、关节力矩上限、月球重力及目标矿石生成方法。

`Lunar` 的数据来自 [USGS Apollo 17 LROC 产品](https://astrogeology.usgs.gov/search/map/moon_apollo_17_lroc_nac_landing_site_orthomosaic_50cm)。数据裁剪、原点、投影、哈希及论文引用见 [数据说明](../assets/terrain_data/apollo17/README.md)。近场插值网格约 0.12 m，**源数据仍是 1.5 m 网格**；近场细节和散石均明确标为合成内容。

```text
assets/
  go2_piper_d435_assets/          原始机器人资产与许可证
  terrain_data/apollo17/          真实 GeoTIFF 裁剪、NPZ/PNG、provenance.json
  lunar/Lunar_baseline/           基线地形与矿石 USD
  lunar/Lunar/                    DEM 地形、矿石 USD、细节材质
  lunar/*_variant_*/              参数变体的独立资产
env/
  config/Lunar_baseline.json      基线配置
  config/Lunar.json               真实数据环境配置
  generated/Lunar_baseline.usda   基线场景
  generated/Lunar.usda            真实数据场景
  generated/*.manifest.json      配置、数据来源、惯量与依赖清单
  prepare_lunar_data.py           重新裁剪原始遥感数据
  terrain.py                     DEM 坐标转换后的本地采样与地形选择
  geometry.py                    原程序化地形、凸矿石与惯量积分
  regolith.py                    可复现的合成细节材质
  check_friction.py               Isaac Sim 滑块摩擦验证
  reports/                       验证报告与渲染图
```

所有运行资产通过相对路径引用，整体移动仓库后仍可使用。正常运行和 USD 重建均不联网，也不依赖外部原始机器人目录。旧 `config/lunar.json` 是基线兼容配置；旧 `generated/lunar_scene.usda`、旧 5 kg 导出与原验证报告保留为历史记录，新实验使用上表的两个环境入口。

## 启动、重建与验证

```bash
# 默认环境为 Lunar，暂停供观察
bash env/run.sh
bash env/run.sh --env Lunar_baseline

# 开始运行浮动基座与受力矩限制的关节 PD
bash env/run.sh --env Lunar --simulate

# 选用 Overview 相机
bash env/run.sh --env Lunar --camera overview

# 重建指定环境
bash env/run.sh --env Lunar --build-only
bash env/run.sh --env Lunar_baseline --build-only

# 5 秒物理检查，保存外部视角与机器人 RGB-D
bash env/run.sh --env Lunar --headless --simulate --steps 1200 --validate \
  --capture env/reports/Lunar --sensors

# 数值测试（不启动 Isaac Sim）
~/anaconda3/envs/isaaclab/bin/python -m unittest discover -s env/tests -v

# 实际 PhysX 滑块摩擦校验
~/anaconda3/envs/isaaclab/bin/python env/check_friction.py
```

本机 Isaac Sim Python 为 `~/anaconda3/envs/isaaclab/bin/python`。其他安装可通过 `ISAAC_SIM_PYTHON=/path/to/python` 或 `ISAAC_SIM_PATH=/path/to/isaac-sim` 指定。常规依赖为 Isaac Sim 5.1.0、NumPy、SciPy、Pillow；重新下载/裁剪遥感数据才需要 `requirements-data.txt` 中的 GIS 依赖。

预生成 USD 可以直接用 Isaac Sim File → Open 打开，选择 `/World/Cameras/Close` 或 `Overview`，点击 Play 运行内置关节 PD。启动脚本会检查配置、生成器代码和运行数据摘要，必要时自动重建；生成文件中的手动修改可能被重建覆盖。

## 三个独立实验变量

两个环境使用相同的参数入口：

```bash
bash env/run.sh --env Lunar --slope-deg 5 \
  --foot-static-friction 0.6 --foot-dynamic-friction 0.5 \
  --mass-kg 5 --simulate
```

| 变量 | JSON 配置 | CLI | 语义 |
| --- | --- | --- | --- |
| 坡度 | `terrain.slope_deg` | `--slope-deg` | 沿世界/投影 +X 的角度，单位 °，允许 −20～20 |
| 足地静摩擦 | `robot.foot_static_friction` | `--foot-static-friction` | 足与地面的有效静摩擦系数，非负 |
| 足地动摩擦 | `robot.foot_dynamic_friction` | `--foot-dynamic-friction` | 足与地面的有效动摩擦系数，须不大于静摩擦 |
| 矿石质量 | `ore.mass_kg` | `--mass-kg` | 单位 kg，严格为正；同步重新积分惯量 |

**坡度约定**：`Lunar_baseline` 的中心平面坡度就是输入角度。`Lunar` 在源高程上增加 `x × tan(slope_deg)`；0 表示保留实测起伏，而非水平地面。默认原点附近 ±2 m 的合成后拟合坡度约为 3.54°，总局部坡度和梯度会写入 manifest。输入角度不是各点最终坡度；对照实验需要固定同一数据裁剪与种子。

**足地摩擦约定**：地面及静态散石使用静/动系数均为 1 的参考材料、`multiply` 合成；足端使用输入系数与 `average`。PhysX 的 `multiply` 优先级更高，因此接触对有效系数恰为输入值。地面的 1 是材料合成约定，不代表实测月壤系数。足端变化只修改足端材料，夹爪指垫、机器人其他部件与矿石摩擦保持各自配置。默认仍为原场景的有效 `0.8 / 0.65`。

**质量约定**：`--mass-kg` 保持矿石几何不变，重心和主惯量由闭合凸网格的均匀密度积分计算，惯量随质量同比例变化。manifest 记录有效密度；质量变化不自动代表某种真实矿物。需要固定密度、改变尺寸的实验时同时设置 `ore.dimensions_m` 和质量。月面重量为 `m × 1.62 N`，刚体质量和惯量不按月球重力比例缩小。

CLI 覆盖不会修改配置文件。与默认配置不同的实验自动输出 `<环境名>_variant_<配置摘要>.usda` 和同名资产目录，避免覆盖默认环境；相同配置可复现相同路径。也可指定独立配置及输出：

```bash
bash env/run.sh --config env/config/my_experiment.json \
  --scene env/generated/my_experiment.usda --simulate
```

复制任一配置文件后修改即可；`environment` 必须与地形类型匹配。`--env` 和 `--config` 同时使用时，两者必须一致。

## 物理、初始姿态与地形细节

| 项目 | 默认值 |
| --- | --- |
| 单位与重力 | 米、千克、秒，Z 向上；`(0, 0, -1.62) m/s²` |
| 步进 | `1/240 s`，CPU PhysX / TGS |
| 机器人 | 25 个刚体、20 个活动关节、自碰撞开启 |
| 矿石 | 2 kg，0.18 × 0.065 × 0.15 m，独立动态凸刚体 |
| 地面 | 静态凹三角网格；真实高程同时用于视觉与碰撞 |
| 基线覆盖 | 近场 40 × 40 m，远景 200 × 200 m |
| Lunar 覆盖 | 近场 40 × 40 m，远景 3000 × 3000 m |
| 相机 | 头部与腕部各 640 × 480，名义水平 FOV 69° |

机器人高度根据关节 FK、足端球半径和当前地面高度计算，初始留 8 mm 间隙；矿石留 3 mm 间隙后自由落地。姿态参数位于 `robot.xy_m / yaw_deg / hip_rad / thigh_rad / calf_rad / arm_rad`，夹爪总开口为 `gripper_opening_m`。旋转关节用 rad，偏航及坡度用 °。

`terrain.flat_radius_m` 在基线中定义平面作业区，在 `Lunar` 中仅限制初始机器人与目标的放置范围，不会压平 DEM。`Lunar` 的 `roughness_m` 控制合成高度细节，可设 0；`rock_count / pebble_count` 控制合成散石数量。固定 seed 保证复现。散石为静态障碍，细碎石合并为一个带三角网格碰撞的静态对象；只有 `/World/Ores/Target` 是动态抓取对象。

当前采用刚性 Coulomb 摩擦，不包含月壤沉陷、剪切、颗粒流动或真空热环境；摩擦是待标定的实验参数。太阳之外的 `ViewingFill` 用于观察，可把 `lighting.fill_intensity` 设为 0。

## 后续 WBC 接入位置

| 对象 | USD prim |
| --- | --- |
| 机器人容器 | `/World/Robot` |
| 浮动基座 / articulation root | `/World/Robot/base` |
| 关节 | `/World/Robot/joints/<joint_name>` |
| 四足 | `/World/Robot/{FL,FR,RL,RR}_foot` |
| 指端 | `/World/Robot/link7`、`/World/Robot/link8` |
| 目标矿石 | `/World/Ores/Target` |
| 地面 | `/World/Terrain/Ground` |
| 头部相机 | `/World/Robot/base/HeadCameraOpticalFrame/Camera` |
| 腕部相机 | `/World/Robot/link6/WristCameraOpticalFrame/Camera` |

场景中没有世界固定关节、基座锁定、吸附器或自动抓取约束。默认通过关节 PD 保持站姿。旋转关节配置增益按 Nm/rad 输入，写入 USD 时转换为 Nm/degree；夹爪使用 N/m。Drive 的最大力/力矩逐关节来自资产 `joint_metadata.json`，包括腿部髋/大腿 23.7 Nm、小腿 45.43 Nm、Piper 前三轴 32 Nm/后三轴 8 Nm、指端 10 N。这些是源模型值，应在硬件实验前与实际电机限制统一。

接入力矩型 WBC 时，应将相应 Drive 的 stiffness/damping 清零或明确采用自己的内环，然后按运行时 `robot.dof_names` 映射施加力矩/力；不要假设关节顺序，也不要在每个物理步重置关节或基座位置。运行时的顺序和源文件排列并不相同，示例已按名称初始化。

`runtime.py` 使用显式物理步进，并单独刷新渲染，保证一次控制步恰好对应 `dt_s`。读取足端力时用相同 `dt_s` 将接触冲量换算为 N；关闭 articulation 休眠避免静态站立后接触力读数归零。后续应在一致的地形、摩擦、惯量和力矩限制下比较 WBC 与基线，当前环境验收不等于重载抓取成功率或最大载荷结论。

## 图像与验证输出

`--capture <目录> --sensors` 写入：

- `close_rgb.png`、`overview_rgb.png`：外部视角。
- `head_rgb.png`、`wrist_rgb.png`：机器人相机视角。
- `head_depth_m.npy`、`wrist_depth_m.npy`：float32 光学 Z 深度，单位 m；无穷、NaN 或非正值为无效数据。
- `*_depth_preview.png`：0–3 m 映射到灰度，仅用于查看。
- `capture.json`：图像尺寸、名义内参和深度有效比例。
- `simulation.json`、`contacts.json`：动态检查、基座倾角和四足接触力。捕获图像时冻结物理时间。

相机沿用资产提供的 ROS 光学坐标系：X 右、Y 下、Z 前；USD Camera 子节点绕 X 旋转 180°，转换到 USD 的 -Z 前、+Y 上。当前相机模拟理想针孔 RGB 与光学深度，不模拟 D435 的双目匹配、主动红外、曝光、噪声或实测手眼标定。

`--validate` 要求至少 240 个物理步，检查关节数、有限状态、站立倾角、四足支撑、矿石落地，以及独立无碰撞探针的自由落体加速度。探针仅存在于运行时，不写入 USD。失败返回非零退出状态。

API 依据 [Isaac Sim 5.1 standalone Python](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/python_scripting/manual_standalone_python.html)、[机器人基础教程](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/introduction/quickstart_isaacsim_robot.html) 和本机 5.1.0 Python API 实现。
