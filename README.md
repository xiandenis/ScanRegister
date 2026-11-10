# ScanRegister

架站三维扫描相关的点云算法工程库（CMake 包名：`scan_register`）。

当前版本：`2.0.2` · C++17 · **Ubuntu 22.04 / Windows（VS 2026 + vcpkg）**

**协作**：长期分支 `main`（Linux）、`windows`（Windows）、`lidarstudio`（LidarStudio 对接，与 `windows` 同级）并行开发，按需互合；大改动可用短生命周期 `feat/*`。详见 [`.cursor/rules/dual-platform-git.mdc`](.cursor/rules/dual-platform-git.mdc)。

---

## 当前仓库状态

根 [`CMakeLists.txt`](CMakeLists.txt) 已建立，可按 Preset / 常规 cmake 配置。已迁入基础模块：

| 路径 | 内容 |
|------|------|
| [`CMakeLists.txt`](CMakeLists.txt) | 工程入口：`project(scan_register)`、依赖、子模块、install |
| [`CMakePresets.json`](CMakePresets.json) | Windows VS2026 / clangd + vcpkg |
| [`common/`](common/) | 公共工具、E57 几何、WorkingFrame、内存调度（SHARED） |
| [`file_system/`](file_system/) | 可移植文件系统与通配（SHARED） |
| [`logging/`](logging/) | 进程内统一日志（SHARED，spdlog） |
| [`robust_rough/`](robust_rough/) | 鲁棒粗匹配：`robust_matcher` / `robin` / `pmc` 等（**STATIC** / INTERFACE） |
| [`cmake/`](cmake/) | 依赖探测、compat、`find_package(scan_register)` 安装导出 |
| [`docs/`](docs/) | Hub/Spoke 字典与模块说明 |

### 配置示例（Windows）

```powershell
cmake --preset windows-vs2026-vcpkg
cmake --build build --config Release -j
```

后续模块：落地源码与 `CMakeLists.txt` 后，追加到 [`cmake/InstallExport.cmake`](cmake/InstallExport.cmake) 清单，并按 `rules.md` 登记字典。

---

## CMake 包约定

| 项 | 值 |
|----|-----|
| `find_package` | `scan_register` |
| 导出 namespace | `scan_register::` |
| 辅助 / 宏前缀 | `scan_register_*` / `SCAN_REGISTER_*` |
| 安装 Config | `lib/cmake/scan_register/` |
| 模块产物默认 | **动态库（SHARED）**（见 [`.cursor/rules/shared-library-default.mdc`](.cursor/rules/shared-library-default.mdc)） |

安装范围（`-Scope` / `SCAN_REGISTER_INSTALL_SCOPE`）：

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
