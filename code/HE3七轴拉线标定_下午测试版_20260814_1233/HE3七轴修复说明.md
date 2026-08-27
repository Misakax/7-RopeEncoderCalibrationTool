# HE3 七轴拉线标定修复说明

## 当前状态

- `MathematicalDLL` 已增加 `CalibrationV2`，七轴计算不再调用旧六轴 24 列符号雅可比。
- 上位机的 HE3_GY 七轴路径已切换到 `CalibrationV2`。
- 当前强制启用离线预览模式：禁止连接、上电、回零、运动、读取编码器和写回机器人参数。
- DLL 和离线测试能够用 `Debug | Win32` 构建并通过。
- 上位机 EXE 尚未重新生成，因为本机 Visual Studio 缺少 v143 x86/x64 MFC 组件。

## 安装构建依赖

保存所有文件并关闭 Visual Studio，然后打开 Visual Studio Installer，修改当前 VS 2022 安装，安装：

`适用于最新 v143 生成工具的 C++ MFC（x86 和 x64）`

对应组件 ID：

`Microsoft.VisualStudio.Component.VC.14.44.17.14.MFC`

安装后在本目录运行：

```bat
Build-DebugWin32.cmd
```

只构建并测试算法 DLL 可运行：

```bat
Test-CalibrationV2.cmd
```

## 如何确认新 DLL 确实被加载

1. 在 VS 中选择 `Debug | x86`，重新生成整个解决方案。
2. 确认 EXE 与 DLL 位于同一目录：`Debug`。
3. 在 `CalibrationV2` 入口设置断点。
4. 启动上位机，在 VS“输出”窗口检查类似信息：

```text
[GUI] Offline preview mode. Loaded MathematicalDLL CalibrationV2 <编译日期时间>
[MathematicalDLL] CalibrationV2 entry ...
```

必须同时满足：DLL 文件时间已更新、输出中的 Build ID 已更新、断点能命中。仅看到桌面界面不代表算法已更新。

## 七轴默认辨识参数

HE3_GY 第一版只辨识：

- 长度：`d2、d3、d4、d5、d7`
- 零位：`q2、q3、q4、q5`

固定：所有 `a/alpha`、`d1、d6、q1、q6、q7`。

默认边界：长度修正 `±5 mm`，零位修正 `±1°`；触及或超过边界返回失败，不允许写回。

## 拉线长度补偿

旧代码中的硬编码 `43 mm + 19.5 mm = 62.5 mm` 已移入机器人配置：

```text
15,0,[Rope length offset(mm)] = 62.5
```

上位机读取后转换为米传给 `CalibrationV2`，并在算法报告中打印实际值。

HE3_GY 当前末端数据链保持为：

```text
d7 = 92.7026 mm + Tool offset 49 mm = 141.7026 mm
```

## 离线回放结果

2026-06-09 的 60 点数据已加入自动测试：

- 初始平均拉线残差：约 `28.46 mm`（成功复现计划基线）。
- 固定 45 点训练、15 点验证。
- 全部点残差：`28.9292 -> 24.9238 mm`。
- 验证点残差：`31.1854 -> 27.4279 mm`。
- 返回状态：`-6`，表示参数修正超出 `±5 mm / ±1°` 安全边界。
- `writeBackAllowed = 0`。

这说明新算法能稳定计算并降低残差，但这组实测数据尚未达到安全写回条件。应继续核查数据同步、固定端/工具定义、62.5 mm 补偿和模型参数，不得直接写入机器人。

## 接口兼容策略

- 旧 `Calibration` 导出保留给六轴历史调用。
- 旧入口收到七轴 Cobot 数据时直接拒绝，防止输出看似正常但实际错位的结果。
- 新 `CalibrationV2` 使用普通 C 数组作为 DLL 边界，输出完整 MDH、各轴零位、固定端、前后残差、迭代次数、秩、条件数和状态码。

