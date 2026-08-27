# R7.2.2a 编译修正版

这是对 R7.2.2 的纯编译热修复，不改变标定算法或最终写回契约。

## 编译失败原因

RopeEncoderCalibrationDlg.cpp 原第 1600 行：

```cpp
const double verifyTolerance =
    integerIdn ? 0.01 : std::max(tolerance, 0.00051);
```

MFC/Windows 头文件可能定义 `max` 宏，导致 MSVC 将 `std::max(...)`
错误展开，从而出现：

- C2589: `(`: `::` 右边的非法标记
- C2059: 语法错误 `)`
- C2737: verifyTolerance 必须初始化

## 修复

改为不使用 `max` 标识符：

```cpp
const double verifyTolerance =
    integerIdn ? 0.01 :
    ((tolerance > 0.00051) ? tolerance : 0.00051);
```

行为完全等价。

## 保持不变的最终契约

- 拉线算法只辨识/写回 D2~D5、Alpha1~Alpha6、q2~q6。
- D6 不是拉线标定参数。
- D7 从不发送写命令。
- 若检测到控制器 D6≈93、D7≈93，可经独立人工确认执行 HE3 结构修复 D6→0。
- D7 保持写回前原值。
- bL 只属于测量链，不写 MDH。
- D/Alpha 按控制器 0.001 分辨率写入和回读。
- 1127.13 零点按整数文本写入。

## 编译

解压到：

`C:\Users\1282\Desktop\拉线编码\R7.2.2a_编译修正版`

然后：

```powershell
Set-ExecutionPolicy -Scope Process Bypass
cd "C:\Users\1282\Desktop\拉线编码\R7.2.2a_编译修正版"
.\apply_R7.2.2a_and_build.ps1
```

成功必须看到 `BUILD PASS`。

EXE：

`C:\Users\1282\Desktop\拉线编码\R7.1-Build\bin\Win32\Release\HE3_RopeCalibration_V3_Tool.exe`
