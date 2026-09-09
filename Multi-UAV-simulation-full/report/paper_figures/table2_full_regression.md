# Table II — Full 41-Scenario Regression (vel_lag_tau=0.5)

Source: `~/flights/reg_lag_20260714/verdict_summary.txt` (2026-07-14 batch), pulled from sim host 2026-07-15. 41 scenarios: **35 PASS / 5 LOG-SEEN / 1 FAIL** (40/41 effective pass, matching report §5.H).

| Scenario | #UAV | pos_err (m) | min_spacing (m) | solve_ms | Verdict |
|---|---|---|---|---|---|
| `S0_solo1_circle` | 1 | 0.034 | n/a | 0.876 | PASS |
| `S0_solo1_hover` | 1 | 0.033 | n/a | 2.058 | PASS |
| `S10_cross5_line_fast` | 5 | 0.034 | 2.334 | 1.362 | PASS |
| `S11_cross5_perturbed` | 5 | 0.037 | 2.161 | 1.497 | PASS |
| `S12_cross5_wide` | 5 | 0.036 | 2.965 | 1.488 | PASS |
| `S13_cross5_circle_10min` | 5 | 0.034 | 2.514 | 1.477 | PASS |
| `S14_cross5_killnode` | 5 | 0.034 | 2.781 | 1.168 | PASS |
| `S15_cross5_conservative` | 5 | 13.526 | 2.815 | 2.730 | FAIL |
| `S16_cross5_commsdrop` | 5 | 0.036 | 2.634 | 1.421 | PASS |
| `S17_pair2_perturbed` | 2 | 0.037 | 2.756 | 1.071 | PASS |
| `S18_pair2_wide` | 2 | 0.437 | 3.911 | 0.712 | PASS |
| `S19_trio3_perturbed` | 3 | 0.036 | 4.051 | 0.985 | PASS |
| `S1_solo1_line` | 1 | 0.034 | n/a | 0.791 | PASS |
| `S20_pair2_line_d10` | 2 | 0.036 | 3.460 | 3.328 | PASS |
| `S21_pair2_line_d30` | 2 | 0.033 | 2.612 | 2.015 | PASS |
| `S22_trio3_wide` | 3 | 0.040 | 5.180 | 1.734 | PASS |
| `S23_pair2_killnode` | 2 | 0.033 | 2.678 | 1.493 | PASS |
| `S24_trio3_killnode` | 3 | 0.060 | 4.685 | 1.669 | PASS |
| `S25_trio3_commsdrop` | 3 | 0.035 | 5.119 | 1.253 | PASS |
| `S26_trio3_conservative` | 3 | 0.033 | 5.271 | 1.215 | PASS |
| `S27_solo1_fence_line` | 1 | 8.398 | n/a | 1.890 | LOG-SEEN |
| `S28_solo1_fence_circle` | 1 | 0.036 | n/a | 1.243 | LOG-SEEN |
| `S29_trio3_squeeze_warn` | 3 | 0.120 | 1.044 | 1.346 | LOG-SEEN |
| `S2_pair2_hover` | 2 | 0.033 | 2.804 | 1.030 | PASS |
| `S2_pair2_line` | 2 | 0.036 | 2.811 | 1.396 | PASS |
| `S30_trio3_squeeze_emerg` | 3 | 0.940 | 0.957 | 1.130 | LOG-SEEN |
| `S31_solo1_relinquish_alt` | 1 | 0.135 | n/a | 0.784 | LOG-SEEN |
| `S32_solo1_circle_nosafety` | 1 | 0.034 | n/a | 1.092 | PASS |
| `S33_trio3_circle_nosafety` | 3 | 0.035 | 4.888 | 2.850 | PASS |
| `S34_cross5_circle_nosafety` | 5 | 0.035 | 2.719 | 2.520 | PASS |
| `S3_trio3_circle` | 3 | 0.035 | 5.114 | 1.978 | PASS |
| `S3_trio3_circle_rh` | 3 | 0.037 | 4.674 | 1.231 | PASS |
| `S3_trio3_hover` | 3 | 0.037 | 4.789 | 1.124 | PASS |
| `S3_trio3_line` | 3 | 0.034 | 4.884 | 1.455 | PASS |
| `S4_cross5_line` | 5 | 0.035 | 2.669 | 1.144 | PASS |
| `S5_cross5_circle` | 5 | 0.033 | 2.670 | 1.354 | PASS |
| `S6_cross5_hover` | 5 | 0.033 | 1.930 | 1.172 | PASS |
| `S7_grid9_hover` | 9 | 0.034 | 2.583 | 1.456 | PASS |
| `S7_grid9_line` | 9 | 0.034 | 2.656 | 1.301 | PASS |
| `S8_grid9_circle` | 9 | 0.031 | 2.561 | 1.606 | PASS |
| `S9_star5_line` | 5 | 0.035 | 2.938 | 1.394 | PASS |
