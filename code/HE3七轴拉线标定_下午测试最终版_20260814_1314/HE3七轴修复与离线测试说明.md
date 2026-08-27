# HE3 七轴拉线标定：构建与离线测试说明

## 1. 构建

1. 双击 `RopeEncoderCalibrationTool.sln`。
2. 选择 `Debug | x86`。
3. 执行“生成 → 重新生成解决方案”。
4. 必须成功生成：`MathematicalDLL`、`MathematicalDLLTests`、`RopeEncoderCalibrationTool`，且为 0 个错误。

也可以运行根目录的 `Build-DebugWin32.cmd`。若 PowerShell 执行策略阻止脚本，可直接在 Visual Studio 中构建。

## 2. 离线测试

运行：

```text
Debug\MathematicalDLLTests.exe
```

最后必须显示：

```text
ALL TESTS PASSED
```

测试覆盖：

- 无误差数据应保持零残差；
- 注入 HE3 长度和关节零位误差后应恢复；
- d7 +2 mm 和 q2 零位误差应被识别；
- 训练点和留出验证点均应通过 1 mm 门限；
- q6/q7 不可辨识请求应被解释并拒绝；
- d6/d7 重复敏感度只能保留一个；
- 样本不足必须返回错误；
- HE3 实际 60 姿态应具备足够秩和可接受条件数；
- 旧的不匹配数据必须被安全边界拒绝，且不得允许写回。

当前 HE3 实际 60 姿态的合成真值审计结果：

```text
active robot parameters = 9
rank (3 anchor coordinates + 9 robot parameters) = 12
condition number ≈ 296.7
training MAE after calibration ≈ 0.0000095 mm
held-out validation MAE after calibration ≈ 0.000014 mm
```

这些结果证明当前 60 姿态和算法在自洽模型下能识别所选参数；它不是实机精度证明。

## 3. 当前数据契约

- 关节反馈以度保存，进入 DLL 前转换为弧度；关节方向不会重复乘两次。
- 配置中的本体 `d7` 不含工具偏置；`Tool offset = 49 mm` 通过 `options.toolOffset[2]` 单独传入。
- 拉线文件保存原始伸出增量；`Rope length offset = 62.5 mm` 通过 `options.ropeLengthOffset` 单独传入。
- 编码器换算系数位于 `RopeEncoderCalibration/Config/RobotType/HE3_GY-cfg.txt` 第 16 项，当前为原程序的 `284.94 counts/mm`。
- 调用 DLL 前强制检查正好收到 60 组七轴反馈、60 个拉线值都可解析且有效长度均为正数。

## 4. 现场模式边界

当前允许受控采集和计算，但不允许自动写回七轴参数。实机操作必须按照 `HE3实机测试操作说明.md` 执行。现场完成后必须保存机器日志、关节文件、拉线文件和 CalibrationV2 报告，才能继续判断实际残差来源。

