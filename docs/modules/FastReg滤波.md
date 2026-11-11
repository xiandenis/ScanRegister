# FastReg 滤波

| 目标 | 类型 | 说明 |
|------|------|------|
| `fastreg_filter` | **SHARED** | 深度图差分滤波（异常率 / 重叠率 / 深度有效率） |

## 链接关系

```text
fastreg_filter
  ├── logging
  ├── opencv_world（OpenCV core/imgproc/imgcodecs）
  ├── Eigen3
  └── OpenMP
```

消费：`target_link_libraries(foo PRIVATE fastreg_filter)`。不在默认伞目标 `scan_register` 内；`party` / `lidarstudio` 默认不装，仅 `full` 导出。

## API 契约（摘要）

| 类 | 输入深度类型 | 邻域 | 阈值 |
|----|--------------|------|------|
| `PanoramaFilter` | `CV_16UC1` | 3×3（水平环绕） | 角向 `sin(π/cols)` 相关 |
| `SpatialPerspectiveFilter` | `CV_32FC1` | 5×5（`BORDER_WRAP`） | `max(threshold, depth*tolerance)` |

公共入口：`difference(src, tgt, diff_path="")` → 成功后可读 `getAbnormalRate` / `getOverlapRate` / `getDepthValidRate`。

## 字典

[`../dictionaries/FastReg滤波_Dictionary.md`](../dictionaries/FastReg滤波_Dictionary.md)
