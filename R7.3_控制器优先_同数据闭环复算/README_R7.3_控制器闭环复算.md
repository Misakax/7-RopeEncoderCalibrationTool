# R7.3 控制器优先 + 同数据闭环复算

## 目的

本版按现场要求做两件事：

1. `开始`前的算法输入不再强制公共 MDH。
   - 控制器实时回读有效：弹窗默认推荐“控制器”；
   - 也可人工选择公共 `HE3_GY-cfg.txt`；
   - 控制器不可用时才自动回退公共模型。
2. 原“复输入检验”改为“控制器闭环复算”。
   - 不运动；
   - 不重新采集82点；
   - 不写机器人；
   - 自动读取当前控制器 D / Alpha / Zero；
   - 自动加载 `./Result` 中最新的第一轮 `V3Diagnostic_*.csv`；
   - 使用其中原82点、第一轮最终 q0；
   - 把当前控制器实际 D/Alpha + 第一轮 q0 作为第二次初值；
   - 再运行同一算法，检查第一轮结果是不是稳定固定点。

## 为什么 q0 必须单独作为 q0Initial

第一轮82点是在旧 Zero 表下采集的。第一轮 q0 后来已经被换算成 ZeroCount 写入控制器。
为了拿“旧82点”做同数据复算，必须把第一轮 q0 作为 q0Initial 重新加入模型。

不能把第一轮 q0 塞进 theta 再令 q0Initial=0，因为 R7.2 的激光先验是对“绝对 q0”建立的；
那样第二次先验会错误地把 q0 再拉一次。

R7.3 已修正这一点。

## 复算输出

每次点击“控制器闭环复算”都会新建：

- `./Result/R73ControllerReplay_YYYY_MM_DD_HH_MM_SS.csv`
- `./Result/R73ControllerReplay_YYYY_MM_DD_HH_MM_SS.log`

并在右侧日志打印它们的绝对路径。

CSV 包含：

- 第一轮记录 MAE / MAX
- 当前控制器修复结果作为初值时的 MAE / MAX
- 第二次再次优化后的 MAE / MAX
- 第二次额外需要的最大 D / Alpha / q 修正量
- 每轮完整参数
- 当前控制器 D / Alpha / Zero
- 第一轮 q0 seed
- 稳定性 PASS / FAIL

当前工程稳定性门限：

- 控制器初值几何 MAE <= 1.0 mm
- 复算最终几何 MAE <= 1.0 mm
- 第二次最大附加 D <= 0.20 mm
- 第二次最大附加 Alpha <= 0.10 deg
- 第二次最大附加 q <= 0.10 deg

这是“同一批82点的算法固定点/稳定性检验”，不是独立激光 TCP 绝对精度认证。

## 编译

解压到：

`C:\Users\1282\Desktop\拉线编码\R7.3_控制器优先_同数据闭环复算`

PowerShell：

```powershell
Set-ExecutionPolicy -Scope Process Bypass
cd "C:\Users\1282\Desktop\拉线编码\R7.3_控制器优先_同数据闭环复算"
.\apply_R7.3_and_build.ps1
```

成功应看到 `BUILD PASS`。

## 你现在的验证操作

你已经有第一轮 `V3Diagnostic_...csv`，而第一轮修复参数也已经持久化到控制器。

因此编译运行 R7.3 后：

1. 连接机器人；
2. 不点“开始”，不运动；
3. 直接点“控制器闭环复算”；
4. 程序读取当前控制器并自动加载上一轮82点；
5. 看右侧结论和 `R73ControllerReplay_*.csv`。

如果当前控制器 D/Alpha 与最新第一轮诊断文件不匹配超过控制器量化误差，R7.3 会拒绝混用数据。
