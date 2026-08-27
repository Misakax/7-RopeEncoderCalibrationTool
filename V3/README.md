# HE3 拉线编码器七轴解析标定 V3

这是在 V1 可信测量约定和 MDH 变换基础上完成的七轴解析标定核心。V3 不使用
V2 的数值差分 Jacobian、LM 或 Huber 作为量产数学核心。

## 先看结论

- V1 的 `GetJ2JTrans`、49 mm 工具长度处理、62.5 mm 拉线固定补偿继续保留。
- 七轴参数布局统一为 `[a1..a7 | d1..d7 | alpha1..alpha7 | q01..q07]`。
- 位置 Jacobian 由矩阵乘积法精确求导，不含手写六轴下标。
- 拉线固定端先用 V1 相邻球方程初始化，再作为 3 个解析未知量与机器人参数共同求解。
- DLL 公共接口只使用整数、`double` 和普通数组，避免跨 DLL 传递 Eigen 对象导致
  `0xC0000005`。
- 49 mm 已折入 `d7=0.1417026 m`；62.5 mm 只加到拉线读数，二者不能重复相加。

## 打开和编译

用 Visual Studio 2022 打开：

`HE3_RopeCalibration_V3.sln`

量产上位机当前是 32 位，优先选择：

`Release | x86`

主要输出位于：

`bin\Win32\Release\CalibrationV3Analytic.dll`

## 项目作用

| 项目 | 作用 |
|---|---|
| `CalibrationV3Analytic` | 七轴/N轴解析标定 DLL |
| `V3GradientCheck` | 将解析位置 Jacobian 与中心差分对照，仅用于测试 |
| `V3SyntheticTest` | 注入已知 `d5=+0.8 mm`，验证能否恢复真值 |
| `V3OfflineTest` | 读取关节/拉线 txt，离线查看残差和参数可观测性 |
| `V3MeasurementAudit` | 检查 49/62.5 长度约定、点位覆盖和奇异值 |
| `V1BinaryReplay` | 只用于回放 V1 原 DLL，不属于 V3 量产 DLL |
| `V1_Reference` | V1 源码只读参考 |

## 已通过的代码级验证

1. 七轴解析位置 Jacobian 最大绝对误差约 `7.3e-10`。
2. `q7` 对轴线上挂点的位置导数为 0，符合当前可自由旋转工装的物理结构。
3. 合成数据注入 `d5=+0.8 mm`，恢复约 `+0.7999999997 mm`。
4. V1 原 DLL 回放 HE3 七轴数据会得到数百毫米错误，证明旧六轴硬编码不能直接用于七轴。

## 仍需实机确认

代码正确不等于已经证明实机达到 1 mm。量产放行前必须确认：

1. 当前机器人配置与采集数据属于同一台机器人、同一版本；
2. 上位机传入的是实际反馈关节角，单位为弧度；
3. 原始拉线值先从 mm 转 m，再加 `0.0625 m`；
4. `d7` 已含 49 mm，不能把 49 mm 再加到拉线值；
5. 量产究竟允许写回哪些 `d` 和零位参数；
6. 使用独立验证点证明平均/最大误差达到项目指标。

8 月 17 日数据是 V2 失败样本，只允许用于复现 V2，不是 V3 验收数据。

## 推荐阅读顺序

1. `docs\01_证据与版本边界.md`
2. `docs\02_实机验证与点位设计.md`
3. `docs\03_上位机接入.md`
4. `CalibrationV3Analytic\CalibrationV3Analytic.h`
5. `CalibrationV3Analytic\CalibrationV3Analytic.cpp`

