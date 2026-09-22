"""rev5 (2026-09-22) CPU YOLOv8n cost under a saturated shared host.
Wall-clock FPS is meaningless when the container gets ~0.3 core, so we measure CPU TIME per frame
(time.thread_time for the calling thread, torch intra-op threads = 1) — the core-milliseconds a frame costs.
FPS on one dedicated core = 1000 / core_ms; SMT/cache contention can only inflate core_ms (conservative).
Also reports wall time for reference. Throwaway container (amr-fleet-system:wf-final), not amr_dev."""
import time, os, json, sys
t_imp = time.perf_counter()
import numpy as np, torch
torch.set_num_threads(1); torch.set_num_interop_threads(1)
from ultralytics import YOLO
res = {"torch": torch.__version__, "import_wall_s": round(time.perf_counter() - t_imp, 1), "threads": torch.get_num_threads()}
rng = np.random.default_rng(0)
frame = rng.integers(0, 255, (480, 640, 3), dtype=np.uint8)
m = YOLO("yolov8n.pt")
net = m.model.fuse().eval()
def measure(fn, n, warm):
    for _ in range(warm): fn()
    cpu, wall = [], []
    for _ in range(n):
        c0, w0 = time.thread_time(), time.perf_counter(); fn()
        cpu.append((time.thread_time() - c0) * 1e3); wall.append((time.perf_counter() - w0) * 1e3)
    return dict(core_ms_median=round(float(np.median(cpu)), 1), core_ms_p90=round(float(np.percentile(cpu, 90)), 1),
                wall_ms_median=round(float(np.median(wall)), 1), n=n)
for sz in (320, 640):
    x = torch.rand(1, 3, sz, sz)
    with torch.inference_mode():
        res[f"forward_fp32_{sz}"] = measure(lambda: net(x), n=15, warm=3)
    print(json.dumps({f"forward_fp32_{sz}": res[f"forward_fp32_{sz}"]}), flush=True)
    res[f"predict_full_{sz}"] = measure(lambda: m.predict(frame, imgsz=sz, device="cpu", verbose=False), n=15, warm=3)
    print(json.dumps({f"predict_full_{sz}": res[f"predict_full_{sz}"]}), flush=True)
res["host_loadavg"] = os.getloadavg()
print(json.dumps(res, indent=1), flush=True)
