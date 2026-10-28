# 资产目录

- `go2_piper_d435_assets/`：用户提供的完整 Go2 + Piper + D435 资产，保留原文件、许可证和校验清单。
- `terrain_data/apollo17/`：USGS 分发的 Apollo 17 LROC DEM、正射影像的真实裁剪数据、离线运行用 NPZ/PNG 与来源记录。
- `lunar/Lunar_baseline/`：原程序化场景的地形与矿石 USD。
- `lunar/Lunar/`：真实 DEM 驱动的地形、矿石 USD 和生成的月壤细节材质。
- `lunar/Lunar*_variant_*/`：不同实验参数的独立生成资产。

用 `bash env/run.sh --env Lunar --build-only` 或 `--env Lunar_baseline` 重建。正常启动与重建使用仓库内数据，不需要联网。重新裁剪原始遥感数据使用 [prepare_lunar_data.py](../env/prepare_lunar_data.py)，详见 [数据来源](terrain_data/apollo17/README.md)。
