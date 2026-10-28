# Lunar 与 Lunar_baseline 验证记录

验证日期：2026-10-22。本机 Isaac Sim 5.1.0，NVIDIA RTX 3090 Ti；CPU PhysX / TGS，物理步长 1/240 s。

## 数值与资产检查

- 11 项数值测试通过：[原始输出](numerical_tests.txt)。覆盖 GeoTIFF 与 NPZ 像元一致性、东西/南北方向、地理参考、源文件哈希、实测起伏保留、范围检查、独立参数及矿石惯量。
- 原基线地面与 Lunar_baseline 的 132,496 个顶点逐点相同；默认动态结果亦保持一致。
- 默认 Lunar 与 5° 变体的地形 X/Y 完全一致，Z 差符合 x × tan(5°)。
- 矿石由 2 kg 改为 5 kg 后，三项主惯量均为原值的 2.5 倍。
- 两个环境及组合变体的 25 个刚体、20 个活动关节、相机光轴、力矩上限和实际绑定的摩擦材料检查通过。
- 将每个场景全部 USD/纹理依赖复制到仓库外临时目录后重新打开，均无缺失依赖；Lunar 使用 4 个 USD 层和 3 张材质纹理。详见 [结构报告](structure.json)。

## 实际物理仿真

三个场景均运行 1200 步，即 5 秒，完成四足支撑、站立状态、矿石落地及自由落体重力检查。

| 环境 | 附加坡度 | 足地静/动摩擦 | 矿石质量 | 最大基座倾角 | 结果 |
| --- | --- | --- | --- | --- | --- |
| Lunar_baseline | 0° | 0.8 / 0.65 | 2 kg | 0.5110° | PASS |
| Lunar | 0° | 0.8 / 0.65 | 2 kg | 4.9909° | PASS |
| Lunar_variant | 5° | 0.6 / 0.5 | 5 kg | 9.0990° | PASS |

Lunar 的坡度输入为 0 时仍保留天然地形。所有场景测得自由落体加速度约 1.6200003 m/s²，四足末段平均竖直支撑力均为正。

- [Lunar_baseline 仿真记录](Lunar_baseline/simulation.json) / [预览](Lunar_baseline/close_rgb.png)
- [Lunar 仿真记录](Lunar/simulation.json) / [预览](Lunar/close_rgb.png) / [RGB-D 采集](Lunar/capture.json)
- [5°、低摩擦、5 kg 组合记录](Lunar_variant/simulation.json) / [预览](Lunar_variant/close_rgb.png)

## 接触摩擦校验

用实际 PhysX 材料、单位参考地面和滑块，在月球重力下根据减速度 a / g 测量动摩擦系数。此项独立核对接触对的合成结果。

| 输入动摩擦 | 滑块质量 | 实测动摩擦 | 结果 |
| --- | --- | --- | --- |
| 0.1 | 1 kg | 0.10002984 | PASS |
| 0.5 | 1 kg | 0.50004323 | PASS |
| 0.5 | 5 kg | 0.50004323 | PASS |

详见 [friction.json](friction.json)；可用 Isaac Sim Python 运行 env/check_friction.py 复现。

## 复现

```bash
python -m unittest discover -s env/tests -v
bash env/run.sh --env Lunar_baseline --headless --simulate --steps 1200 --validate --capture env/reports/Lunar_baseline
bash env/run.sh --env Lunar --headless --simulate --steps 1200 --validate --capture env/reports/Lunar --sensors
bash env/run.sh --env Lunar --slope-deg 5 --foot-static-friction 0.6 --foot-dynamic-friction 0.5 --mass-kg 5 --headless --simulate --steps 1200 --validate --capture env/reports/Lunar_variant
python env/check_friction.py
python env/validate_scene.py --scene env/generated/Lunar_baseline.usda env/generated/Lunar.usda
```

其中 python 应为 Isaac Sim 环境解释器。上述结果验证场景、参数和 PD 预览；目标矿石未被抓持，不构成 WBC 性能、最大抓取质量或真实月壤物理精度的结论。可配置的坡度/摩擦范围也不等于当前 PD 姿态的稳定范围。
