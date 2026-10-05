"""rev5 (2026-09-22) CPU-fallback FPS and fleet CPU budget from the measured per-frame CPU cost
(bench/cpu_pipeline.txt: letterbox + fused YOLOv8n FP32 imgsz 320 + decode + NMS, 1 intra-op thread,
calling-thread CPU time on a saturated host -> upper bound on cost). Budget refs: multi_robot.md s7."""
import json, re
txt = open("bench/cpu_pipeline.txt").read()
res = json.loads(txt[txt.index("{\n"):txt.rindex("}") + 1])
c, c90 = res["pipeline_320"]["core_ms_median"], res["pipeline_320"]["core_ms_p90"]
print(f"pipeline_320 core-ms median {c} p90 {c90} (pre {res['pre_320']['core_ms_median']}, forward {res['forward_320']['core_ms_median']}, post {res['post_320']['core_ms_median']}); loadavg {res['host_loadavg'][0]:.0f}")
print(f"FPS per dedicated core: median {1000/c:.1f}, p90 {1000/c90:.1f}")
print(f"10 FPS single stream: {c/100:.2f} cores at perfect scaling; with 2 intra-op threads needs parallel efficiency >= {c/200:.2f}")
budget, total_10fps_plan, yolo_prepost = 25.6, 23.5, 3.0    # multi_robot.md s7 (80 % of 32 threads; plan total; 0.6 x 5)
for hz in (10, 5):
    add = 5 * c * hz / 1000; tot = total_10fps_plan - yolo_prepost + add
    print(f"5 robots CPU inference @ {hz} Hz: {add:.2f} cores -> host total {tot:.1f} vs budget {budget} ({100*tot/32:.0f} % of 32 threads) -> {'FITS' if tot <= budget else 'EXCEEDS'}")
