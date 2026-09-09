# Consensus 非预测对照实验数据（论文 §6.5 Table III 行 (e)）

memoryless 分布式位移一致性律 vs MPC，cross5、同 §6.2b 风扰协议（2N@100, 260s / calm 200s）。

- `default/` — 名义增益 kL=1.5/kC=0.8：wind n=5（峰峰 0.886±0.035m，plan_ratio 0.93 不饱和）+ calm n=5（稳态 0.073±0.009m）。**表 III 行 (e) 数据来源。**
- `highgain_kL6p3/` — 高增益 kL=6.3≈k_p(§5.6)/kC=3.4，wind n=5：深度饱和（plan_ratio~10.5×、~80k clips/run）**仍有界不发散**（峰峰 1.544±0.134m，各机 |x|≤4.9m）。§6.5 "非仅保守增益假象" 论证来源。
- `probe_kL1p75/` — 中间档 kL=1.75 单次（峰峰 0.771m），扫描过渡点记录。

分析口径：`tools/repeat_stats.py --group X <...wind...run[1-5].csv> --wind-window 110 258`（calm 用 100 198）。
高增益档用 `run_one_trial.sh` 的 `RECORD_ON_TIMEOUT=1`（增益高时松散振荡过不了就绪门，但有界，须录制才能量化）。
对照：MPC-legacy(a) 峰峰 median~153m 发散 / MPC-fix(d) 0.61m。
