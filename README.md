# real_time_registration

面向架站三维扫描的**半实时点云配准**工程库：从测站初始化、粗全局配准，到位姿图连通与 MST 全局位姿传播，统一由顶层 [`RealTimeManager`](real_time_registration/real_time_manager.hpp) 编排。可选通过 ROS 2 服务对外提供能力。

当前版本：`2.0.2` · C++17 · **Ubuntu 22.04 / Windows（VS 2026 + vcpkg）**

**协作**：长期分支 `main`（Linux）、`windows`（Windows）、`lidarstudio`（LidarStudio 对接，与 `windows` 同级）并行开发，按需互合；大改动可用短生命周期 `feat/*`。详见 [`.cursor/rules/dual-platform-git.mdc`](.cursor/rules/dual-platform-git.mdc)。

---

## 功能概览

| 能力 | 说明 |
|------|------|
| 工程管理 | 工程目录 / `info` / `cache`、边表落盘与回读 |
| 测站生命周期 | 加站、初始化检查/计算、删站 |
| 配准 | 两站粗+精配准（`autoRegistration`）；一站式加站+配准（`registerScan`） |
| 全局位姿 | 边表连通分量 + MST 传播；导出 `global_matrix.mat` |
| ROS 2 | Service 封装：工程 / 注册 / 删站 / 更新全局矩阵（**Linux / colcon**） |

```text
点云 LAS/PCD/...
        │
        ▼
registration_initializer ──► rough_global_reg
        │                           │
        └──────────► RealTimeManager ◄── pose_graph_tool / graph
                            │
              ┌─────────────┴─────────────┐
              ▼                           ▼
     边表 align_result.txt      global_matrix.mat（导出）
              │
              ▼（可选，Linux）
     ROS 2 real_time_registration_node
```

---

## 仓库结构

```text
real_time_registration/
├── real_time_registration/   # RealTimeManager（业务编排入口）
├── registration_initializer/ # 测站初始化
├── rough_global_reg/         # 粗全局配准
├── fine_registration/        # 精配准（FastGICP）
├── basic_icp/                # 基础 ICP（Fast/Robust/Sparse）
├── small_gicp/               # 轻量并行 GICP/VGICP（vendored）
├── pose_graph_tool/          # 位姿图分组与全局位姿
├── global_matrix_file/       # mat 读写
├── registration_type/        # ScanInfo / AlignResult 等
├── logging/                  # 日志（动态库，进程内统一单例）
├── ros2/                     # ROS 2 msgs + 服务节点（Linux）
├── cmake/                    # 依赖与 install/export
├── scripts/                  # Windows 编译 / clangd 辅助脚本
├── CMakePresets.json         # Windows VS2026 + vcpkg 预设
├── vcpkg.json                # vcpkg 依赖清单
└── docs/                     # 字典、模块说明、使用手册
```

头文件按模块目录包含，例如：

```cpp
#include <real_time_registration/real_time_manager.hpp>
#include <registration_type/registration_type.hpp>
```

---

## 依赖

| 类型 | 依赖 |
|------|------|
| 构建 | CMake ≥ 3.16（Windows 用 VS 2026 生成器建议 CMake ≥ 4.2），C++17 |
| 第三方 | Eigen3、PCL、PDAL、OpenCV、spdlog、TBB、FLANN、RapidJSON、Threads；OpenMP（可选） |
| ROS 2（可选，Linux） | Humble：`rclcpp`、`geometry_msgs`、`tf2_eigen`、`colcon` |

### Ubuntu

```bash
sudo apt install build-essential cmake \
  libeigen3-dev libpcl-dev libpdal-dev libopencv-dev \
  libspdlog-dev libtbb-dev libflann-dev
```

### Windows

1. 安装 **Visual Studio 2026**（含“使用 C++ 的桌面开发”）
2. 安装 **CMake ≥ 4.2**（系统 PATH 或 VS 自带均可）
3. 安装 [vcpkg](https://vcpkg.io/)，设置环境变量 `VCPKG_ROOT`
4. 按 [`vcpkg.json`](vcpkg.json) 安装依赖（示例）：

```powershell
$env:VCPKG_ROOT = "D:\install\vcpkg"   # 按本机路径修改
& "$env:VCPKG_ROOT\vcpkg.exe" install --triplet x64-windows `
  eigen3 tbb pcl pdal opencv4 spdlog flann rapidjson lz4
```

---

## 编译（算法库）

> `logging` 为**动态库**（`logging.dll` / `liblogging.so`），以保证各静态模块共享同一套日志单例。运行测试或部署时请保证能加载该库。

### Linux

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

可选安装，供 `find_package(real_time_registration)` 使用：

```bash
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=$PWD/install_cmake
cmake --build build --target install
```

测试可执行文件示例：

```bash
./build/real_time_registration/real_time_manager_test \
  /path/to/las_dir \
  /path/to/project
```

### Windows（推荐）

需已设置 `VCPKG_ROOT`。一键脚本（编译成功后默认 **install** 到仓库根下 `install_party\<Config>\`，已 gitignore；可用 `-Prefix` 改根路径，仍会再挂一层 `Release` / `Debug`）：

```powershell
.\scripts\build_windows.ps1              # 默认 Release + party install → <repo>\install_party\Release
.\scripts\build_windows.ps1 -Config Debug # → <repo>\install_party\Debug
.\scripts\build_windows.ps1 -Prefix D:\libs\real_time_registration  # → …\Release 或 …\Debug
.\scripts\build_windows.ps1 -Scope full
.\scripts\build_windows.ps1 -Scope lidarstudio          # LidarStudio 产品头白名单 + 链接闭包库
.\scripts\build_windows.ps1 -Scope full -Publish        # 装完发到 third-party-library 的独立分支并打 tag
.\scripts\build_windows.ps1 -Scope lidarstudio -Publish
.\scripts\build_windows.ps1 -SkipInstall # 只编不装
.\scripts\build_windows.ps1 -ConfigureOnly
```

安装范围（`-Scope` / `REAL_TIME_REGISTRATION_INSTALL_SCOPE`）：

| Scope | 头文件 | 库 |
|-------|--------|-----|
| **party**（默认） | Session 面：`registration_type`、`real_time_registration`、`logging`（+ `normal_estimate.hpp`） | `real_time_manager` 链接闭包 + SHARED |
| **full** | 全量减去内部叶子（pmc / xenium / matcher_*） | 全部库目标 |
| **lidarstudio** | 产品白名单（cloud_*、logging、octree、overlap_estimation、registration_type、rough/fine、auto_registration、file_system、point_cloud_io、common）+ fine 传递头（registration_report、basic_icp） | 白名单根目标链接闭包（更深依赖只装 `.lib`/`.dll`） |

发布仓（多库、一库一分支；本库分支 `real_time_registration`，标签 `real_time_registration/vX.Y.Z`）：

- 远程：`http://192.168.123.124/realone/desktop/lidarprocess/third-party-library.git`
- 本地目录仓：`..\third-party-library`（`main` 仅目录说明）
- 本地库工作区：`..\third-party-library-pkgs\real_time_registration`
- 也可单独：`.\scripts\publish_windows_lib.ps1 -Scope lidarstudio`（需已对应 scope 安装；每次发布先升版本号）
- full 安装**不**包含头：`pmc`、`xenium`、`matcher_tsl`、`matcher_points`、`matcher_kdtree`、`matcher_hnswlib`
- 第三方（PCL/Eigen/…）不进安装树；消费端用 vcpkg + `find_dependency`

```text
git clone -b real_time_registration --single-branch http://192.168.123.124/realone/desktop/lidarprocess/third-party-library.git real_time_registration
git checkout real_time_registration/v2.0.2
```

或使用 CMake Preset（生成器：`Visual Studio 18 2026`）：

```powershell
cmake --preset windows-vs2026-vcpkg -DCMAKE_INSTALL_PREFIX=$PWD/install_party/Release
cmake --build build --config Release -j
cmake --install build --config Release --prefix $PWD/install_party/Release
```

产物默认在 `build\bin\Release\`（含 `logging.dll` 与各测试 `.exe`），例如：

```powershell
.\build\bin\Release\real_time_manager_test.exe D:\data\las D:\data\project
.\build\bin\Release\real_time_manager_test.exe D:\data\las D:\data\project 3   # 仅前 3 站；第 3 参数 <=0 或省略=全部
```

一键取前 N 站链式测试（直接读 DataDir，不暂存 LAS_firstN；并导出 CloudCompare 4×4）：

```powershell
.\scripts\test_realtime_windows.ps1 -DataDir D:\data\caijian_sation -ProjectDir "D:\data\project\精配准测试" -Stations 3
.\scripts\test_realtime_windows.ps1 -DataDir D:\data\caijian_sation -ProjectDir "D:\data\project\精配准测试" -Stations 0 -Build   # 0=全部；缺 exe 时加 -Build
```

clangd / IntelliSense 可参考本地 `.clangd`（仓库提供 [`.clangd.example`](.clangd.example)），并用：

```powershell
.\scripts\gen_compile_commands.ps1
```

常用 CMake 选项：

| 选项 | 默认 | 说明 |
|------|------|------|
| `BUILD_REAL_TIME_MANAGER_TEST` | ON | 编译 `real_time_manager_test` |
| `BUILD_CENTER_NEAREST_OCTREE_UNIT_TEST` | ON | 八叉树单测 |
| `REAL_TIME_REGISTRATION_INSTALL` | 顶层为 ON | 是否生成 install/export |
| `REAL_TIME_REGISTRATION_INSTALL_SCOPE` | `party` | `party` / `full` / `lidarstudio` |
| `BUILD_SHARED_LIBS` | OFF | 其余模块以静态库为主（`logging` 固定为 SHARED） |
| `REAL_TIME_REGISTRATION_BUILD_ROS2` | OFF | 根 CMake 不编 ROS 2；Windows 上强制关闭 |

链接方式：

- 伞目标：`real_time_registration::real_time_registration`
- 或直接：`real_time_manager`（会传递依赖 `logging` 动态库）

---

## C++ 快速使用

```cpp
#include <real_time_registration/real_time_manager.hpp>
#include <registration_type/registration_type.hpp>

Ddx::RealTimeManager* mgr = Ddx::RealTimeManager::getInstance();
mgr->updateProjectFolder("/path/to/project");

Ddx::ScanInfo scan;
scan.name_ = "station_001";
scan.lasFn_ = "/path/to/a.las";
scan.sensor_ = Ddx::SENSOR_STAND;

Ddx::AlignResult result;
mgr->registerScan(scan, "", result);                    // 首站：原点
// mgr->registerScan(scan2, "station_001", result);     // 后续站

std::vector<std::map<std::string, Eigen::Matrix4d>> subgraphs;
mgr->updateGlobalMatricesFromEdges(subgraphs);
mgr->saveGlobalMatrixToLocalFile();
```

更完整的模块说明见 [`docs/modules/实时配准管理.md`](docs/modules/实时配准管理.md)。

---

## ROS 2 服务（可选，Linux）

将 `RealTimeManager` 封装为服务节点，其他节点无需链接算法库。Windows 本分支不配置 ROS 2，请使用算法库 CMake 编译。

```bash
source /opt/ros/humble/setup.bash
# 或使用仓库脚本：./ros2/build_ros2.sh
colcon build \
  --paths ros2/real_time_registration_msgs ros2/real_time_registration_ros2 \
  --build-base ros2_ws/build \
  --install-base ros2_ws/install \
  --symlink-install
source ros2_ws/install/setup.bash

ros2 launch real_time_registration_ros2 real_time_registration.launch.py \
  default_project_path:=/path/to/project
```

| 服务 | 作用 |
|------|------|
| `~/update_project` | 设置工程目录 |
| `~/register_scan` | 一站式配准（`scan_name` + `ref_scan_name` + `las_path`） |
| `~/delete_scan` | 删除测站 |
| `~/update_global_matrices` | 重算并导出全局矩阵 |

详细接口、CLI、集成示例与常见问题：

**[`docs/ROS2_RealTimeManager使用手册.md`](docs/ROS2_RealTimeManager使用手册.md)**

---

## 文档

| 路径 | 内容 |
|------|------|
| [`docs/total.md`](docs/total.md) | 文档总索引 |
| [`docs/点云算法总览_Dictionary.md`](docs/点云算法总览_Dictionary.md) | 跨模块依赖速查 |
| [`docs/dictionaries/`](docs/dictionaries/) | 各领域算法字典 |
| [`docs/modules/`](docs/modules/) | 模块 API / 设计说明 |
| [`docs/plan/`](docs/plan/) | 实施与优化计划归档 |
| [`rules.md`](rules.md) | 文档生成与归档规则 |

---

## 设计要点（摘要）

- **半实时**：一站扫完后同步处理，非异步流水线。  
- **相对边 vs 全局位姿**：边表存 src→tgt 相对变换；全局在 `m_name2GlobalMatrixMap`。  
- **增量 vs MST 重算**：`registerScan` 按参考站增量写全局；`updateGlobalMatricesFromEdges` 按字典序最小站为原点重算，绝对系可能不同。  
- **单例**：进程内一份工程状态；ROS 节点使用单线程 executor。  
- **日志**：`logging` 为动态库，避免多静态库各持一份 logger。

---

## License

未单独声明时，请与项目维护方确认使用条款。
