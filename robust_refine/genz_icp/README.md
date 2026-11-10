# GenZ-ICP（本仓库迁入）

整库迁入自 [cocel-postech/genz-icp](https://github.com/cocel-postech/genz-icp) commit `674ccce2705df012a29a99d3b8ef0e6c1cbc0129`（2026-10-13）。

仅包含上游 `core` + `pipeline`。不包含 ROS / Python / metrics。

规格：[`docs/superpowers/specs/2026-10-22-genz-icp-port-design.md`](../docs/superpowers/specs/2026-10-22-genz-icp-port-design.md)

## License

MIT。见本目录 `LICENSE`（上游版权声明保留）。Sophus 头文件见 `3rdparty/sophus/LICENSE`。

## Citation

```bibtex
@ARTICLE{lee2024genzicp,
  author={Lee, Daehan and Lim, Hyungtae and Han, Soohee},
  journal={IEEE Robotics and Automation Letters (RA-L)},
  title={{GenZ-ICP: Generalizable and Degeneracy-Robust LiDAR Odometry Using an Adaptive Weighting}},
  year={2025},
  volume={10},
  number={1},
  pages={152-159},
  doi={10.1109/LRA.2024.3498779}
}
```

## 模块边界

- 静态库目标：`genz_icp`
- 依赖：本库 Eigen、TBB、`matcher_tsl`、`pcl_common`；Sophus 1.22.11 头文件仅在本模块 `3rdparty/sophus`
- **不**链接进伞库 `real_time_registration`（算法库目标可被 `fine_registration` 链接）
- FineRegistration：`AlgorithmType::kGenZIcp`；默认 `GenZIcpParams::auto_voxel_size=true`

Include：

```cpp
#include <genz_icp/genz_icp.hpp>            // 站间：genz_icp::GenZICP（对标 TrimICP）
#include <genz_icp/pipeline/GenZICP.hpp>    // 里程计：pipeline::GenZICP（上游原名，站间勿用）
```

| 类 | 用途 |
|---|---|
| `genz_icp::GenZICP` | 两站配准：目标建哈希地图，源云 + 初值 `align` |
| `genz_icp::pipeline::GenZICP` | 序列里程计（deskew / 自适应阈值 / 局地图） |

预设：`GenZICP::Config::Indoor()`（0.05 / 1.0）、`Outdoor()`（0.25 / 1.0）。不对输入再体素下采样。

## 成对程序

CLI 薄封装，内部调用 `genz_icp::GenZICP`。

```text
genz_icp_pair <target> <source> [init.txt]
    [--voxel-size 0.05] [--max-distance 1] [--kernel <2/3>]
    [--min-range <m>] [--max-range <m>]
```

输出 \(T\) 满足 \(p_{\text{target}} = T\, p_{\text{source}}\)。  
**不对输入点云再体素下采样**（室内 / 已有 `voxel_*` 缓存）；`--voxel-size` 只设哈希地图格边长。默认 Indoor：`--voxel-size 0.05`、`--max-distance 1`。

## 烟测

```powershell
cmake --build build --config Release --target genz_icp_compile_test genz_icp_pair
.\build\bin\Release\genz_icp_compile_test.exe
```
