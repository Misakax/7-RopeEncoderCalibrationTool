# HE3 拉线标定 R7 网页端 AI 交接

## 当前任务

HE3 是 7 轴机器人。工具用拉线编码器在多姿态下测量末端到固定端的距离，辨识 Craig MDH 参数与关节零位。算法主链为：

`theta + MDH -> GetPee/FK -> 末端XYZ -> GetLambda -> 拉线固定点 -> 长度残差 -> 解析参数Jacobian -> 最小二乘 -> 参数修正`。

## 已确认的模型契约

- 算法内部使用 Craig MDH。
- 控制器 `1101.12` 是旧式 Alpha 顺序，读取后右移一槽转为 Craig MDH。
- 控制器 D7 是约 93 mm 的本体末端长度；拉线工具 ToolOffset 为 49 mm。算法有效 D7 约为 142 mm，写回控制器时不能重复加 49 mm。
- 激光方案候选开放参数：D2~D5、Alpha1~Alpha7、q2~q6。
- 当前拉线求解：D2~D5、Alpha1~Alpha6、q2~q6；Alpha7/q7 对轴向测量点是零列，保持机械/控制器值。
- A1~A7、D1/D6/D7、q1/q7、Beta 保持。

## R6.5 日志暴露的根因

R6.3 旧六轴写回曾执行 `1101.11[6] = 93.000`，导致控制器存在 `D6=93, D7=93`。正常结构应为 `D6≈0, D7≈93`。R6.5 读取该错误模型后 MAE 约 56 mm，而拉线重复性只有约 0.0014 mm，所以不是传感器噪声。R6.5 安全门拦截了保存。

## R7 新增逻辑

1. 连接时只读控制器快照，不覆盖算法输入。
2. 点击“开始”时必须选择：控制器真实 MDH / 原始本地 MDH / 取消。
3. 原始 MDH 是加载配置后立即保存的不可变 `MatrixXd` 快照，不会被控制器回读污染。
4. 选择后在右侧打印完整 MDH 及来源，再次确认才能运动。
5. 控制器输入与写回候选均新增结构检查：`|D1|<=5 mm`、`|D6|<=5 mm`、`75<=D7<=110 mm`。
6. Build ID：`HE3-V3-CRAIG-MDH-ANALYTIC-LASER-POLICY-20260826-R7-MDH-SOURCE-SELECT`。

## 当前验证状态

- Release x86 全解决方案编译成功，0 错误。
- Synthetic D5/q5 恢复测试 PASS。
- q6/q7 结构零列测试 PASS。
- 解析 Jacobian 梯度检查 PASS，最大绝对导数误差约 `7.34e-10`。
- 没有在本次修改中启动 GUI、连接机器人或写回参数。

## 继续工作时必须索要的新证据

下一次现场运行后，请同时索要：

1. `log/YYYY_M_D_log.txt`；
2. `Result/V3Diagnostic_*.csv`；
3. `data/HE3_GY_R5SampleStats_*.csv`；
4. `Joint/HE3_GY_Joint_*.txt`；
5. 用户在 R7 弹窗中选的是“控制器真实 MDH”还是“原始 MDH”。

不要只看 GUI 上的 MAE，必须核对 `input_model_source`、D1~D7、ToolOffset、安全门和逐点残差。

## 编辑优先级

- 主要 GUI/数据流：`RopeEncoderCalibrationDlg.cpp/.h`。
- 主要数学核心：`CalibrationV3Analytic.cpp/.h`。
- 配置解析：`FileOperation.cpp/.h`。
- 控制器通信：`QKMLinkComm.cpp/.h`。
- 不要恢复旧 V1/R6.3 只写 12 个六轴结果槽位的逻辑。

