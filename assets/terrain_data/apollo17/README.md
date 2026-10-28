# Apollo 17 月面数据

数据来自 **Technical University Berlin / DLR / LROC Team**，由 **USGS Astrogeology** 分发：
[Moon Apollo 17 LROC NAC Landing Site Orthomosaic 50cm](https://astrogeology.usgs.gov/search/map/moon_apollo_17_lroc_nac_landing_site_orthomosaic_50cm)。

论文引用：Haase, I., Wählisch, M., Gläser, P., Oberst, J., & Robinson, M. S. (2019). *Coordinates and Maps of the Apollo 17 Landing Site*. Earth and Space Science, 6, 59–95. [doi:10.1029/2018EA000408](https://doi.org/10.1029/2018EA000408)。按数据发布页要求引用作者。

## 仓库中实际使用的数据

| 文件 | 内容 |
| --- | --- |
| `dem_crop.tif` | 原生 1.5 m 网格的高程裁剪；2049 × 2049，保留源像元与地理参考 |
| `ortho_crop.tif` | 原生 0.5 m/像素的正射影像裁剪；6145 × 6145 |
| `dem.npz` | 从 DEM 无损提取的 float32 高程、局部 X/Y 轴；Y 按向北递增 |
| `ortho_reference.png` | 正射影像缩小至 2048 × 2048，用于低频色调参考 |
| `provenance.json` | 精确源 URL、ETag、裁剪窗口、投影、获取日期与每个文件的 SHA-256 |

选取的场景原点为 **30.771679708°E，20.190811734°N**，对应投影坐标 `(2040.0, 612253.5) m`。这是本工程选定的裁剪中心，不代表对机器人部署点的实测标定。原点源高程为 `-2626.199951171875 m`，基准为半径 **1,737,400 m** 的月球参考球。

投影为月球球面 Transverse Mercator，中央经线 30.7°E。仿真 X/Y 为投影东/北方向，Z 为减去原点高程后的相对高程；使用米制距离，不把经纬度直接当作米。裁剪保留约 3.07 km 见方的原始数据，默认渲染其中 3 km 见方。

源文件：

- [APOLLO17_DTM_150CM.TIFF](https://planetarymaps.usgs.gov/mosaic/Apollo17/APOLLO17_DTM_150CM.TIFF)
- [APOLLO17_ORTHOMOSAIC_50CM.TIFF](https://asc-astropedia.s3.us-west-2.amazonaws.com/Moon/Apollo/Traverse/Apollo17/ancillary/APOLLO17_ORTHOMOSAIC_50CM.TIFF)

脚本通过 HTTP Range 读取窗口，保留 GeoTIFF 裁剪，未下载完整约 950 MB 的源文件。SHA-256 校验对象是仓库中的裁剪/转换文件；远端源文件身份另用 URL、ETag、长度和最后修改时间记录。

## 场景中的处理与合成内容

DEM 用双线性插值生成近场细网格，远景降低网格密度。**插值不会提高原始测量分辨率**；没有抹平中央区域或人为添加陨石坑。原点附近 ±2 m 范围的拟合天然坡度约 3.57°，叠加默认小尺度合成起伏后约 3.54°。

以下内容属于本工程合成，并非该地点的实测复原：

- 默认幅度参数 2 mm 的近场高度细节；固定随机种子，可设 `terrain.roughness_m=0` 关闭。
- 月壤颗粒颜色、法线及粗糙度材质。
- 静态散石、带碰撞的细碎石，以及独立动态抓取矿石。
- 实验配置的附加坡度、灯光和摩擦参数。

正射影像经过平滑和对比度限制后，只用于地形低频色调；原始轨道影像含照明和阴影，**此处理不是光度校正，也不生成实测反照率**。接触采用刚体 Coulomb 摩擦，不模拟月壤沉陷。

正常启动、重建均离线。重新裁剪需要额外 GIS 依赖：

```bash
python -m pip install -r env/requirements-data.txt
python env/prepare_lunar_data.py
```

更换区域时使用新的 `--output`，再把 `Lunar.json` 的 `terrain.data_dir` 指向新数据目录。数据缺失、校验不符或查询超出覆盖区会报错，不会回退到程序化地形。
