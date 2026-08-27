# R7.2.2 最终写回契约修正

本补丁建立在 R7.2.1 上，**不修改两阶段 + 8 台激光先验数学求解器**。只修正写回语义和闭环验证。

## 最终契约

### 拉线标定允许写回
- D2 ~ D5
- Alpha1 ~ Alpha6
- q2 ~ q6（q6仅在可观测时）

### 永远不是拉线标定量
- D6
- D7
- bL

其中：
- `bL` 只校准拉线测量链，永不写 MDH。
- `D6=0` 是 HE3 厂家 DH 结构定义，不是拉线算法计算出来的参数。
- 控制器若出现 `D6≈93, D7≈93`，只识别为旧状态的 **HE3 结构异常**。
- 结构修复 `D6: 93 -> 0` 必须单独弹窗确认。
- `D7` 始终保持控制器写回前值（该机型应约 93 mm），程序全过程不发送 D7 写命令。

拉线算法内部：
`D7_effective = controller D7 (93 mm) + ToolOffset (49 mm) = 142 mm`

这不等于向控制器写 D7=142。

## R7.2.2 相比 R7.2.1 的实质修改

1. 删除 `candidateD[5] = 0`。
   D6 不再进入“拉线标定候选数组”。

2. 候选日志只列出 D2~D5 / Alpha1~6 / q2~6。
   D6/D7 单独显示为“控制器结构状态”。

3. 写回分两个阶段：
   - 阶段A：拉线标定参数写回（绝不写 D6/D7）
   - 阶段B：若用户明确确认，执行独立 HE3 结构修复，只写 D6=0

4. 两个阶段都通过逐项回读后，才执行一次 `21.23 save`。

5. 保存后再次完整回读并核验：
   - D2~D5 = 量化后的候选
   - Alpha1~6 = 量化后的候选
   - q2~q6 = 整数候选
   - 若结构修复：D6=0
   - D7 = 写回前原值，必须未变化

6. CSV 明确记录：
   - `d6_is_rope_calibration_parameter,0`
   - `d6_structural_repair_required`
   - `d6_structural_repair_applied`
   - `d7_written,0`

## Build ID

`HE3-V3-CRAIG-MDH-ANALYTIC-TWOSTAGE-LASER-PRIOR-FINAL-WRITEBACK-20260826-R7.2.2`

## 编译

把整个文件夹放到：

`C:\Users\1282\Desktop\拉线编码\R7.2.2_最终写回契约修正`

保持已有可编译工程：

`C:\Users\1282\Desktop\拉线编码\R7.1-Build`

PowerShell：

```powershell
Set-ExecutionPolicy -Scope Process Bypass
cd "C:\Users\1282\Desktop\拉线编码\R7.2.2_最终写回契约修正"
.\apply_R7.2.2_final_and_build.ps1
```

看到 `BUILD PASS` 后运行：

`C:\Users\1282\Desktop\拉线编码\R7.1-Build\bin\Win32\Release\HE3_RopeCalibration_V3_Tool.exe`

第一次保存后先不要运动。请检查日志是否依次出现：

1. `拉线标定参数写回开始...此阶段不写D6/D7`
2. `拉线标定参数逐项写入/回读通过；到此为止D6/D7均未写`
3. 若原控制器 D6=93/D7=93：
   `独立HE3结构修复开始（不是拉线标定结果）：仅写D6=0；D7...不发送D7写命令`
4. `21.23 save返回成功`
5. `R7.2.2最终闭环通过`

最后控制器 D 应类似：

`0, 48.x, 492.x, -120.x, 316~317.x, 0, 93`

并且日志/CSV应明确 `d7_written=0`。
