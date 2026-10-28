# 本工程的 Isaac Sim 5.1 训练入口

已新增 **Go2_Piper_Lunar_Tracking**（Go2 + Piper + 月面，移动与末端位置轨迹跟踪），并将原 train.py / play.py 入口切换到 Isaac Sim / Isaac Lab 后端。

请先阅读 [迁移与运行说明](isaaclab/README.md)。从 Go2Piper_Lunar 根目录使用 `bash scripts/train.sh --headless`。源 Gym 任务保留用于参考；新任务不依赖 Isaac Gym，也不要求 W&B 登录。

以下为上游项目说明和论文引用。

# Deep Whole Body Control [CoRL 2022]
This repository contains a reference PyTorch implementation of the paper:

<b>Deep Whole-Body Control: Learning a Unified Policy for Manipulation and Locomotion</b> <br>
[Zipeng Fu*](https://zipengfu.github.io/),
[Xuxin Cheng*](https://chengxuxin.github.io/),
[Deepak Pathak](https://www.cs.cmu.edu/~dpathak/) <br>
Conference on Robot Learning (CoRL), 2022 <br>
[[Website](https://manipulation-locomotion.github.io/)], [[Paper](https://arxiv.org/abs/2210.10044)]


## Reference (BibTex)

```
@inproceedings{fu2022deep,
  author    = {Fu, Zipeng and Cheng, Xuxin and Pathak, Deepak},
  title     = {Deep Whole-Body Control: Learning a Unified Policy for Manipulation and Locomotion},
  booktitle = {Conference on Robot Learning ({CoRL})},
  year      = {2022},
}
```
