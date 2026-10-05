import time, os, numpy as np, torch
from ultralytics import YOLO
torch.set_num_threads(int(os.environ.get("TH", "4")))
f = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
m = YOLO("yolov8n.pt")
for sz in (320, 640):
    for _ in range(3): m.predict(f, imgsz=sz, device="cpu", verbose=False)
    ts = []
    for _ in range(10):
        t0 = time.perf_counter(); m.predict(f, imgsz=sz, device="cpu", verbose=False); ts.append((time.perf_counter()-t0)*1e3)
    print(f"threads={torch.get_num_threads()} imgsz={sz}: median {np.median(ts):.1f} ms, p90 {np.percentile(ts,90):.1f} ms, loadavg {os.getloadavg()[0]:.1f}", flush=True)
