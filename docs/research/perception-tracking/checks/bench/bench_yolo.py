"""YOLOv8n inference latency benchmark (throwaway container, not amr_dev).
GPU: FP16 PyTorch (no TensorRT runtime in image), batch 1 and 5, imgsz 640.
CPU: FP32 PyTorch, imgsz 640/320, torch threads 2/4 (host is shared and loaded -> pessimistic).
Measures model forward only (letterboxed tensor already on device) and the full ultralytics predict()
call on numpy frames (includes pre/post-processing on CPU)."""
import time, sys, os, json
import numpy as np, torch
from ultralytics import YOLO

res = {"torch": torch.__version__, "cuda": torch.cuda.is_available()}
if torch.cuda.is_available():
    res["gpu"] = torch.cuda.get_device_name(0)
frames = [np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8) for _ in range(5)]

def timeit(fn, n=60, warm=15):
    for _ in range(warm):
        fn()
    if torch.cuda.is_available():
        torch.cuda.synchronize()
    ts = []
    for _ in range(n):
        t0 = time.perf_counter(); fn()
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        ts.append((time.perf_counter() - t0) * 1e3)
    ts = np.array(ts)
    return dict(median_ms=round(float(np.median(ts)), 2), p95_ms=round(float(np.percentile(ts, 95)), 2))

mode = sys.argv[1] if len(sys.argv) > 1 else "gpu"
if mode == "gpu" and torch.cuda.is_available():
    m = YOLO("yolov8n.pt")
    net = m.model.fuse().to("cuda").half().eval()
    for b in (1, 5):
        x = torch.rand(b, 3, 640, 640, device="cuda").half()
        with torch.inference_mode():
            res[f"gpu_fp16_fused_forward_b{b}"] = timeit(lambda: net(x), n=200, warm=30)
    m2 = YOLO("yolov8n.pt")
    res["gpu_predict_b1_full"] = timeit(lambda: m2.predict(frames[0], imgsz=640, half=True, device=0, verbose=False))
    res["gpu_predict_b5_full"] = timeit(lambda: m2.predict(frames, imgsz=640, half=True, device=0, verbose=False))
    res["gpu_mem_alloc_MB"] = round(torch.cuda.max_memory_allocated() / 2**20, 1)
else:
    for th in (2, 4):
        torch.set_num_threads(th)
        m = YOLO("yolov8n.pt")
        for sz in (640, 320):
            res[f"cpu_t{th}_predict_{sz}"] = timeit(lambda: m.predict(frames[0], imgsz=sz, device="cpu", verbose=False), n=25, warm=5)
res["host_loadavg"] = os.getloadavg()
print(json.dumps(res, indent=1))
