# HE3 拉线标定 R7.1 网页端 AI 交接

## 核心契约

R7.1 固定使用公共 `Config/RobotType/HE3_GY-cfg.txt` 作为每次采集的算法输入，不再提供控制器 MDH 输入选项。控制器参数只用于写回前后快照和回读验证。R7.1 增广 condition 门限为 `3000`（R7 原为 2500），满秩仍要求 15 个机器人活动列 + 3 个 anchor 列 + 1 个 bL 列全部可辨识。

Craig MDH 与 ToolOffset 规则不变：控制器 D7 约 93 mm，ToolOffset 49 mm，算法有效 D7 约 142 mm；写回控制器时不能重复加 49 mm。

## R7.1 新增变量 bL

测量模型：

`L_measured = ||P-lambda|| + bL`

残差：

`r = L_measured - ||P-lambda|| - bL`

联合求解变量包含活动机器人参数、固定端 `lambda_x/y/z` 与独立标量 `bL`。`bL` 的 Jacobian 列恒为 `+1`。

`bL` 是测量链 nuisance parameter：

- 不属于 `[a,d,alpha,q0]` 的 `4N` 参数向量；
- 不返回到 `outDelta4N`；
- 不映射到 D/Alpha/Zero；
- 写回 CSV 必须有 `rope_bias_written_to_mdh,0`。

## GUI/日志必须核对

GUI 同时显示：

- 拉线零偏 `bL`；
- 去零偏前 MAE；
- 去零偏后几何 MAE。

诊断 CSV 应包含 `rope_bias_mm`、`mae_before_bias_mm`、`geometry_mae_after_bias_mm`、`rope_bias_corrected_mm` 和 `measurement_nuisance_parameters,anchor_x|anchor_y|anchor_z|bL`。

Build ID 必须是：

`HE3-V3-CRAIG-MDH-ANALYTIC-LASER-POLICY-20260826-R7.1-PUBLIC-MDH-ROPE-BIAS`

## 参数开放策略

继续沿用 R7 激光包络策略：D2~D5、Alpha1~Alpha6、q2~q6（q6 由可观测性决定）。A、D1/D6/D7、q1/q7、Beta、Alpha7 保持。

## 当前数据证据

交付中的 R7 82 点标定前残差全部为正：均值/MAE `2.936891 mm`。仅减去该统一均值后 MAE `0.658670 mm`。R7 几何修正后的残差均值 `2.464771 mm`，再去统一均值后的 MAE `0.566915 mm`。

Python 对 R7 原 DLL 数学链的逐行镜像回放能复现交付 CSV 到约 `1e-5 mm`。随后直接编译修改后的 R7.1 C++ 数学核心并调用真实导出 API 回放本组旧数据，结果为：`bL=+3.196252891 mm`、几何 `MAE=0.568765711 mm`、`MAX=3.323237069 mm`、`condition=2636.268409`、rank 15/15（增广 19/19）。

该 C++ 回放同时给出 `D3=493.879471 mm`、`D4=-117.000211 mm`，超过现有激光包络，因此旧数据预计仍会被安全门拒绝写回。现场结论必须以 Windows R7.1 重编后的重新采集为准；MAE<1 mm 不代表自动允许写回。

## 下一次现场运行需索要

1. `log/YYYY_M_D_log.txt`
2. `Result/V3Diagnostic_*.csv`
3. `Result/R71Writeback_*.csv`（如执行保存）
4. `data/HE3_GY_R5SampleStats_*.csv`
5. `Joint/HE3_GY_Joint_*.txt`
6. GUI 三项：bL / 去零偏前 MAE / 去零偏后几何 MAE

优先检查 `input_model_source` 是否明确为公共 HE3_GY 配置、`rope_bias_written_to_mdh` 是否为 0、rank/condition 是否仍通过，以及少数高残差点是否仍集中在 19/61/62/71 等姿态。

## 本次代码验证

当前 Linux 环境直接编译了修改后的数学 DLL 源码与测试程序：Synthetic D5/q5/bL、q6/q7 结构零列全部 PASS；bL 注入 `2.75 mm` 恢复为 `2.750000017 mm`，同时 D5 伪修正约 `-1.24e-10 mm`；GradientCheck 最大绝对导数误差 `7.336e-10`。Windows MFC EXE/DLL 仍需在完整 Windows 源码树中 Release|x86 重编。
