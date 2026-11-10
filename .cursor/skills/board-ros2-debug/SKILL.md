---
name: board-ros2-debug
description: >-
  SSH 联调开发板：编译 ROS2（ros_ws_ssh）并对数据文件夹做全量链式 register_scan 测试。
  Use when the user mentions 开发板、板子、远程编译、开发板编译 ros2、开发板测试数据、
  remote_dev.sh、ros_ws_ssh、或 Laser@192.168.110.88 联调。
---

# 开发板 ROS2 联合调试

权威口语约定与字典：[`docs/dictionaries/ROS2实时配准_Dictionary.md`](../../../docs/dictionaries/ROS2实时配准_Dictionary.md)  
短规则：[`.cursor/rules/ros2-build-test.mdc`](../../rules/ros2-build-test.mdc)  
入口脚本：仓库根 [`remote_dev.sh`](../../../remote_dev.sh)

## 默认连接

| 项 | 默认 |
|----|------|
| SSH | `Laser@192.168.110.88` |
| 远端仓库 | `/home/Laser/ddx_work_spaces/real_time_registration` |
| 编译产物 | `ros_ws_ssh/` |
| 工程建议 | `/home/Laser/ddx_work_spaces/project/<工程名>` |

可用环境变量覆盖：`REMOTE_HOST` / `REMOTE_USER` / `REMOTE_PATH` / `REMOTE_ROS_DISTRO` / `REMOTE_WS_NAME` / `ROS_DOMAIN_ID`

SSH 需 `required_permissions: ["all"]`（读 `~/.ssh`、出网）。

## 工作流

### A. 开发板编译 ros2

触发：「开发板编译 ros2」「开发板编译」等。

```bash
./remote_dev.sh build-ros2
# 可选：./remote_dev.sh build-ros2 -j8
```

产物：远端 `ros_ws_ssh/install`。勿用本机 `./ros2/build_ros2.sh`（那是 `ros2_ws/`）。

### B. 开发板测试数据

触发：「开发板测试数据」「在开发板进行测试」等，且用户给出：

1. **数据路径文件夹**（点云目录）
2. **工程文件夹**（可新建；一般为 `.../project/<工程名>`）

```bash
./remote_dev.sh test-ros2 \
  --data-dir <数据路径文件夹> \
  --project-dir <工程文件夹>
```

约定：

- 默认测数据目录下一层全部 `.las/.slas/.pcd/.ply`（不递归、按名排序）
- 顺序：`update_project` → 链式 `register_scan`（首站 ref 空）→ `update_global_matrices`
- 站名默认用文件名去后缀（`--name-mode stem`）；仅调试可用 `--name-mode index` → `station_001…`
- 需先编译：加 `--build`
- **实时网页默认关闭**；仅当用户同时说「开报告网页 / 实时网页 / 同时开网页」时加 `--dashboard`（板端 `0.0.0.0:8765`，浏览器 `http://192.168.110.88:8765/`；测完保留）
- 单站 LAS 可能数分钟；`block_until_ms` 留足（常 ≥15–40 min），用 `AwaitShell` 跟进度
- **测试结束后脚本会自动打印详细效率**（`ros2/report_ros2_efficiency.py`：TicToc 拆解 + 点数 + 瓶颈占比）。Agent 向用户汇报时直接引用该输出，无需再手工翻 `node.log`

缺路径时先问用户，不要猜本地 `~/data`。

**voxel_size_ 估计改动**：必须测满三组回归（`test_las` / `trimble_cheku` / `foshan_lingnan_las`），汇报须同时含 **voxel 组成 + 粗配准效率**（`reg_s`/`flann`/`feat_*`），见 [`.cursor/rules/voxel-size-estimate-regression.mdc`](../../rules/voxel-size-estimate-regression.mdc) 与 [`docs/plan/深度曲率体素估计.md`](../../../docs/plan/深度曲率体素估计.md)。

### C. 其它

```bash
./remote_dev.sh check   # 连通性
./remote_dev.sh ssh     # 交互登录
./remote_dev.sh cmd 'ls'
./remote_dev.sh build-lib          # 算法库 party 安装（默认）
./remote_dev.sh build-lib --full   # 全量头/库
```

等价：`./scripts/remote_build_install_lib.sh`。默认前缀 `/home/Laser/ddx_work_spaces/install_party/real_time_registration`；party 装 `registration_type` + `real_time_registration` + `logging` 头，以及 `libregistration_type.so` / `liblogging.so`。

## 效率报告

`./remote_dev.sh test-ros2` 成功后会输出 `=== EFFICIENCY REPORT ===`，含：

- 每站 `init_s` / `reg_s` / `load_s` / `root_filter` / `voxel_3`
- init 各子阶段秒表
- `info/*.info` 原始点 / Root / 体素点数
- 平均瓶颈占比

手工补打：

```bash
python3 ros2/report_ros2_efficiency.py \
  --node-log /tmp/rtr_ssh_test_*/node.log \
  --project-dir <工程文件夹>
```

## 不要做

- 手写冗长 colcon / 多段 `ros2 service call`（除非用户要求逐步 CLI）
- 把本机 `ros2_ws/` 与远端 `ros_ws_ssh/` 混用
- 提交 `__pycache__/`、`ros_ws_ssh/` 等构建/缓存产物
- 开发板测完后省略效率表（用户约定：必须展示详细效率）
