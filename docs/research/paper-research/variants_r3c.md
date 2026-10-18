# R3-C 변형 고정 (prereg_r3c §2) — E1·E3 실행 전 커밋

- 선택 (tuning_selection_r3.json, 커밋된 HEAD): DWA_R3.chosen_z = **Z09**, DWB DWB_D31, MPPI MPPI_P30, RPP RPP_R27.
- 선택 설정 (CC-DWA ≡ dwa_R3): research {"reset_persist_cycles": 6, "reset_model": true, "reset_model_theta": 0.1, "goal_extension_dist": 1.0, "blocked_detour_shift": 1.0, "model_delay": 0.01, "model_tau": 0.06, "accel_hold": true}, top {"velocity_weight": 0.6}.
- 변형은 표의 키만 덮어쓴다 (`scripts/params_r3c.py`). "= CC-DWA" 는 기본값을 채운 유효 설정이 선택과 같아 돌지 않는다.
- 블록 YAML·chain cfg 원본: `/home/CAPYI_AI/paper_research/runs/r3c/variants/` (봉인).

| id | 팔 | 원고 행 | 덮어쓰기 | 상태 | ROS 블록 YAML sha256 | chain cfg sha256 |
| --- | --- | --- | --- | --- | --- | --- |
| CC-DWA | dwa_R3 | CC-DWA | `{}` | run | `9f9deecbcd40f9c4215f1a6ba540bcb3eed954f4403d5256216467069f223840` | `c2278b2e268192b3ec670ae930c65dcb4ea218ecde502bd3e4a3e67f4d70e2fa` |
| V-meas | dwa_v_meas | 측정 기준 창 | `{"top:window_center_measured": true}` | run | `e9f898b8a1552c0e5c4d8afd8a10e53d8eddfdde613551c6709261fa018a6854` | `ed7c419a0f462e03974dbab447e018e51de606fd058cffbfc8412362ecba5223` |
| V-thr | dwa_v_thr | 임계 재설정 | `{"research:accel_hold": false, "research:reset_model": false}` | run | `a24fea0dae39e9e22d15c12cea5e81979a4e72f2646a9420bc616b46436fc9f8` | `357b2a4ab78481df9e1dc645a42b7bc2eb8c448444731b872d6dda11894ee711` |
| V-mdl-n | dwa_v_mdl_n | 모형 (명목값) | `{"research:accel_hold": false, "research:reset_model": true, "research:reset_model_theta": 0.1, "research:model_delay": 0.04, "research:model_tau": 0.08}` | run | `67d046e243e65aba92781a1d5c2dad2178024f92b1e7272d301de3ca752840b5` | `5cf3efed3fdf2eea38c82d0b06ee9491793b14b838b65bbe94904d36ae6b4f39` |
| V-mdl-i | dwa_v_mdl_i | 모형 (식별값) | `{"research:accel_hold": false, "research:reset_model": true, "research:reset_model_theta": 0.1, "research:model_delay": 0.01, "research:model_tau": 0.06}` | run | `3a490dccbb001f88e0abba452a15249056ae53f25d98e3a982733975a43d7da6` | `06e7c70bc75e329bd3819290e0bbf2ce80541e16e349989d0c482563f23e00e6` |
| V-K0 | dwa_v_K0 | 지속 조건 끔 | `{"research:reset_persist_cycles": 0}` | run | `f2f7770a0a18b1e1527d79548953115e61645ce97deab3810dcff381e059b4e9` | `4af602988f3e2ac375bd7df8559f52089e45029e2b5dc9eedc9c710d502ffc52` |
| V-noext | dwa_v_noext | 말단 정렬 끔 | `{"research:goal_extension_dist": 0.0}` | run | `358f76994006a5e3a9a64f601a66e24d79ac0b067610b2691dda5f0b259baddc` | `96fa9d4d1627c10a3d224e29e6565d7a5cad56a92b693872672a92f5263f98a6` |
| V-nodet | dwa_v_nodet | 우회 끔 | `{"research:blocked_detour_shift": 0.0}` | run | `443c692ebbbf3d9fd2eb95cc3546ce7e26be4ec955793f16d5ad5097f87fabf5` | `2f052c8ba66912d6f38c40c15ce12390a880ea8d916401653a0458464d00041c` |
| V-G1only | dwa_v_G1only | 유예만 (모형 끔) | `{"research:reset_model": false, "research:accel_hold": true}` | run | `cbd1910b0d212fa2c2d7b8d08b7dfd6c450c724771754c3857c2b131e9dd9bdd` | `7f23ff4976630e876fb9dd32951bafa13c10f2a6b64190c4695d3cf8176e13fe` |
