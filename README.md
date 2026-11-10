# ScanRegister

架站三维扫描相关的点云算法工程库（CMake 包名：`scan_register`）。

当前版本：`2.0.2` · C++17 · **Ubuntu 22.04 / Windows（VS 2026 + vcpkg）**

**协作**：长期分支 `main`（Linux）、`windows`（Windows）、`lidarstudio`（LidarStudio 对接，与 `windows` 同级）并行开发，按需互合；大改动可用短生命周期 `feat/*`。详见 [`.cursor/rules/dual-platform-git.mdc`](.cursor/rules/dual-platform-git.mdc)。

---

## 当前仓库状态

本仓处于**脚手架阶段**：算法模块目录与根 `CMakeLists.txt` 尚未迁入，暂不可完整配置/编译。已就绪：

| 路径 | 内容 |
|------|------|
| [`cmake/`](cmake/) | 依赖探测、compat 辅助、`find_package(scan_register)` 安装导出骨架 |
| [`docs/`](docs/) | 字典 / 模块说明 / 计划归档（含历史能力叙述） |
| [`rules.md`](rules.md) | 文档生成、能力复用与全局登记 |
| [`vcpkg.json`](vcpkg.json) | 第三方依赖清单（`name`: `scan-register`） |
| [`.cursor/rules/`](.cursor/rules/) | 双平台、动态库默认、能力登记等治理规则 |

模块迁入后：在对应目录落地源码与 `CMakeLists.txt`，并把目标名追加到 [`cmake/InstallExport.cmake`](cmake/InstallExport.cmake) 的 party / full / lidarstudio 清单。

---

## CMake 包约定

| 项 | 值 |
|----|-----|
| `find_package` | `scan_register` |
| 导出 namespace | `scan_register::` |
| 辅助 / 宏前缀 | `scan_register_*` / `SCAN_REGISTER_*` |
| 安装 Config | `lib/cmake/scan_register/` |
| 模块产物默认 | **动态库（SHARED）**（见 [`.cursor/rules/shared-library-default.mdc`](.cursor/rules/shared-library-default.mdc)） |

安装范围（`-Scope` / `SCAN_REGISTER_INSTALL_SCOPE`；模块清单当前为空）：

| Scope | 含义 |
|-------|------|
| **party**（默认） | 对外 Session 面：清单内头 + 根目标链接闭包 |
| **full** | 全量库目标与头模块 |
| **lidarstudio** | LidarStudio 产品白名单根目标链接闭包 |

发布仓分支 / 标签（模块可发布后）：`scan_register` · `scan_register/vX.Y.Z`。

常用选项（根 CMake 落地后）：

| 选项 | 默认 | 说明 |
|------|------|------|
| `SCAN_REGISTER_INSTALL` | 顶层为 ON | 是否生成 install/export |
| `SCAN_REGISTER_INSTALL_SCOPE` | `party` | `party` / `full` / `lidarstudio` |
| `BUILD_SHARED_LIBS` | **ON** | 各模块默认动态库 |
| `SCAN_REGISTER_BUILD_ROS2` | OFF | 根 CMake 不编 ROS 2；Windows 上强制关闭 |

---

## 依赖（vcpkg）

见 [`vcpkg.json`](vcpkg.json)：Eigen3、TBB、PCL、PDAL、OpenCV、spdlog、FLANN、RapidJSON、lz4、Ceres、libE57Format 等。第三方不进安装树；消费端用 vcpkg + `find_dependency`。

---

## 文档

| 路径 | 内容 |
|------|------|
| [`docs/total.md`](docs/total.md) | 文档总索引 |
| [`docs/点云算法总览_Dictionary.md`](docs/点云算法总览_Dictionary.md) | 跨模块依赖速查 |
| [`docs/dictionaries/`](docs/dictionaries/) | 各领域算法字典 |
| [`docs/modules/`](docs/modules/) | 模块 API / 设计说明 |
| [`docs/plan/`](docs/plan/) | 实施与优化计划归档 |
| [`rules.md`](rules.md) | 文档生成、复用优先与归档规则 |

新增模块 / 公共方法前：先查字典复用，落地后按 `rules.md` 全局登记。

---

## License

未单独声明时，请与项目维护方确认使用条款。
