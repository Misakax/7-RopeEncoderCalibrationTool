# HE3 七轴拉线标定：修复内容与离线测试说明

## 1. 当前允许做什么

当前版本只允许：

- 编译 EXE、DLL 和测试程序；
- 离线读取已有的 HE3_GY 60 点数据；
- 运行 CalibrationV2；
- 查看标定前后拉线残差、参数状态、秩和条件数；
- 保存离线诊断报告。

当前禁止：

- 连接机器人；
- 自动上电、回零或运动 60 点；
- 把计算结果写回机器人。

原因：真实数据回放仍返回 `CALIBRATION_V2_CORRECTION_OUT_OF_RANGE (-6)`，部分参数触及 ±5 mm / ±1° 安全边界；标定后平均拉线残差仍约为 24.9 mm，且尚无激光跟踪仪独立验证。

## 2. 整体数据流程

```text
HE3_GY-cfg.txt
  -> 轴数、MDH、Tool offset、Rope length offset

HE3_GY.txt + 历史拉线数据
  -> 60 组七轴角度 + 60 个拉线长度

CalibrationV2
  -> 七轴正运动学，计算每个姿态的末端挂线点 XYZ
  -> 用训练点估计拉线固定端 XYZ
  -> 计算“实测长度 - 模型长度”残差
  -> 用数值雅可比计算每个候选参数对残差的影响
  -> 用 SVD 检查参数是否可辨识
  -> 用带阻尼的迭代最小二乘求参数修正
  -> 用独立的 15 个验证点检查是否泛化
  -> 检查 ±5 mm / ±1° 安全边界
  -> 只输出诊断结果，禁止写回
```

## 3. 主要修改位置

### 3.1 新的 DLL 接口

- `MathematicalDLL/MathematicalDLL.h:47`：状态码、参数状态、选项、结果结构体。
- `MathematicalDLL/MathematicalDLL.h:117`：`CalibrationV2` C 数组接口。
- `MathematicalDLL/MathematicalDLL.def:5`：导出 `CalibrationBuildId` 和 `CalibrationV2`。

V2 不再跨 DLL 边界传 Eigen 对象。长度统一为米，角度统一为弧度。

### 3.2 七轴正运动学、数值雅可比和求解器

- `MathematicalDLL/MathematicalDLL.cpp:112`：通用 N 轴正运动学和独立工具偏移。
- `MathematicalDLL/MathematicalDLL.cpp:143`：估计拉线固定端坐标。
- `MathematicalDLL/MathematicalDLL.cpp:212`：通用数值参数雅可比，替代七轴路径中的旧六轴 24 列公式。
- `MathematicalDLL/MathematicalDLL.cpp:255`：SVD、秩和条件数。
- `MathematicalDLL/MathematicalDLL.cpp:378`：V2 入口、参数检查和错误码。
- `MathematicalDLL/MathematicalDLL.cpp:495`：45 个训练点、15 个验证点。
- `MathematicalDLL/MathematicalDLL.cpp:552`：最多 50 次带阻尼迭代和最佳结果回退。
- `MathematicalDLL/MathematicalDLL.cpp:664`：±5 mm、±1° 边界检查。

### 3.3 旧接口安全保护

- `MathematicalDLL/MathematicalDLL.cpp:710`：保留旧 `Calibration`，兼容旧六轴调用。
- `MathematicalDLL/MathematicalDLL.cpp:722`：旧接口遇到七轴 Cobot 时明确拒绝，避免输出假的七轴结果。

### 3.4 上位机中的七轴调用

- `RopeEncoderCalibration/RopeEncoderCalibrationDlg.cpp:1056`：6/7 轴改用 V2。
- `RopeEncoderCalibration/RopeEncoderCalibrationDlg.cpp:1070`：复制完整 5×N MDH，不再遗漏 d2、d3、d7。
- `RopeEncoderCalibration/RopeEncoderCalibrationDlg.cpp:1082`：HE3_GY 默认参数掩码。
- `RopeEncoderCalibration/RopeEncoderCalibrationDlg.cpp:1107`：迭代、条件数、偏移和安全边界配置。
- `RopeEncoderCalibration/RopeEncoderCalibrationDlg.cpp:1124`：实际调用 `CalibrationV2`。
- `RopeEncoderCalibration/RopeEncoderCalibrationDlg.cpp:1157`：保存 V2 离线报告。
- `RopeEncoderCalibration/RopeEncoderCalibrationDlg.cpp:1190`：始终禁止写回。

HE3_GY 第一版只请求：

```text
d2、d3、d4、d5、d7
q2、q3、q4、q5
```

冻结 q1 是因为它与未知固定端坐标存在坐标自由度；冻结 q6/q7 是因为当前轴向挂线点对它们不敏感；冻结 d6 是因为当前数据下 d6 与 d7 的拉线敏感度相同，无法分别辨识。

### 3.5 配置中的偏移

- `RopeEncoderCalibration/Config/RobotType/HE3_GY-cfg.txt:4`：`Tool offset = 49 mm`。
- `RopeEncoderCalibration/Config/RobotType/HE3_GY-cfg.txt:15`：`Rope length offset = 62.5 mm`。
- `RopeEncoderCalibration/FileOperation.cpp:705`：两个毫米配置都转换成米。

V2 将本体 d7 和 49 mm 工具偏移分开建模，62.5 mm 拉线补偿也通过具名配置传入，不再在算法调用前偷偷加到测量数组中。

### 3.6 安全和调试

- `RopeEncoderCalibration/RopeEncoderCalibrationDlg.cpp:36`：`kOfflinePreviewMode = true`。
- `RopeEncoderCalibration/RopeEncoderCalibrationDlg.cpp:443`：离线模式不初始化通信。
- `RopeEncoderCalibration/RopeEncoderCalibrationDlg.cpp:467`：禁用连接、运动、回零和保存按钮。
- `RopeEncoderCalibration/RopeEncoderCalibrationDlg.cpp:1263`：再次阻止写回。
- `RopeEncoderCalibration/RopeEncoderCalibrationDlg.cpp:2158`：关闭窗口时，离线模式不释放从未初始化的 WebSocket 线程。

### 3.7 构建关系

- `RopeEncoderCalibration/RopeEncoderCalibration.vcxproj:263`：上位机引用 MathematicalDLL 项目，保证先生成 DLL 再链接 EXE。
- `Build-DebugWin32.cmd`：完整 Debug Win32 构建入口。

## 4. 第一次测试：自动离线测试

1. 确认 VS 顶部为 `Debug | x86`。
2. 在解决方案资源管理器右键 `MathematicalDLLTests`。
3. 选择“设为启动项目”。
4. 在 `MathematicalDLL/MathematicalDLL.cpp:378` 设置断点。
5. 按 F5。
6. 断点会进入 `CalibrationV2`；按 F5 继续执行全部用例。
7. 控制台最后必须显示：

```text
ALL TESTS PASSED
```

主要用例位于 `MathematicalDLLTests/CalibrationV2Tests.cpp`：

- 无误差合成数据应接近零修正；
- 注入 d7 +2 mm 后应恢复；
- 注入 q2 零位后应恢复；
- q6/q7 应报告不可辨识；
- 同时请求 d6/d7 时只能保留一个；
- 样本不足应返回错误码；
- 真实 HE3_GY 数据应复现约 28.46 mm 初始 MAE；
- 实际数据永远不得允许写回。

## 5. 第二次测试：上位机只启动和关闭

1. 右键 `RopeEncoderCalibrationTool`，设为启动项目。
2. 按 F5。
3. 在 VS“输出/调试”窗口确认：

```text
[GUI] Offline preview mode. Loaded MathematicalDLL CalibrationV2 ...
```

4. 确认连接、开始、回零和保存按钮不可用。
5. 直接关闭窗口。
6. 不应再出现 `this 是 nullptr` 或 `0xC000041D`。

仅出现 `[GUI]` 表示 DLL 已加载但算法还没有被界面调用；出现 `[CalibrationV2]` 才表示算法真正运行。

## 6. 如何读真实回放结果

当前测试输出示例：

```text
status = -6
all-point MAE = 28.9292 mm -> 24.9238 mm
validation MAE = 31.1854 mm -> 27.4279 mm
writeBackAllowed = 0
```

含义：误差有所下降，但仍很大，而且参数修正触及安全边界，因此只能用于定位模型、单位、固定端或采集数据问题，不能作为机器人标定参数。

下一阶段必须补齐：同一台机器人真实的 60 点反馈角、拉线原始值、治具坐标定义、Tool offset/62.5 mm 的物理确认，以及激光跟踪仪基准报告。
