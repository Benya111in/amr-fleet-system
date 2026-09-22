"""rev5 (2026-09-22) CPU cost of the yolo_node CPU-fallback pipeline WITHOUT the ultralytics predict() wrapper:
cv2 letterbox 640x480 -> imgsz, float tensor, fused YOLOv8n FP32 forward, box decode, torchvision NMS.
Metric = calling-thread CPU time (time.thread_time) with torch intra-op threads = 1, so the host's saturation
(loadavg ~90 / 32 threads) does not enter; SMT/cache contention can only inflate it (conservative).
Also logs torch.get_num_threads() to diagnose the predict() wrapper anomaly seen in cpu_cputime.txt."""
import time, os, json
t0 = time.perf_counter()
import numpy as np, torch, cv2, torchvision
torch.set_num_threads(1); torch.set_num_interop_threads(1); cv2.setNumThreads(1)
from ultralytics import YOLO
res = {"import_wall_s": round(time.perf_counter() - t0, 1)}
net = YOLO("yolov8n.pt").model.fuse().eval()
res["torch_threads_after_import"] = torch.get_num_threads()
frame = np.random.default_rng(0).integers(0, 255, (480, 640, 3), dtype=np.uint8)
def letterbox(img, sz):
    r = sz / max(img.shape[:2]); nh, nw = round(img.shape[0] * r), round(img.shape[1] * r)
    out = np.full((sz, sz, 3), 114, np.uint8); top = (sz - nh) // 2
    out[top:top + nh, :nw] = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    return out
def pre(sz):
    im = letterbox(frame, sz)[:, :, ::-1].transpose(2, 0, 1)
    return torch.from_numpy(np.ascontiguousarray(im)).float().div_(255.0).unsqueeze(0)
def post(y, conf=0.35, iou=0.5):
    p = y[0] if isinstance(y, (list, tuple)) else y           # (1, 4+nc, A)
    p = p[0].T                                               # (A, 4+nc)
    sc, cl = p[:, 4:].max(1); k = sc > conf; b = p[k, :4]; sc = sc[k]
    xyxy = torch.cat([b[:, :2] - b[:, 2:] / 2, b[:, :2] + b[:, 2:] / 2], 1)
    return torchvision.ops.batched_nms(xyxy, sc, cl[k], iou)
def measure(fn, n=15, warm=3):
    for _ in range(warm): fn()
    c, w = [], []
    for _ in range(n):
        c0, w0 = time.thread_time(), time.perf_counter(); fn()
        c.append((time.thread_time() - c0) * 1e3); w.append((time.perf_counter() - w0) * 1e3)
    return dict(core_ms_median=round(float(np.median(c)), 1), core_ms_p90=round(float(np.percentile(c, 90)), 1),
                wall_ms_median=round(float(np.median(w)), 1), n=n)
with torch.inference_mode():
    for sz in (320,):
        x = pre(sz); y = net(x)
        res[f"pre_{sz}"] = measure(lambda: pre(sz))
        res[f"forward_{sz}"] = measure(lambda: net(x))
        res[f"post_{sz}"] = measure(lambda: post(y))
        res[f"pipeline_{sz}"] = measure(lambda: post(net(pre(sz))))
        print(json.dumps({k: v for k, v in res.items() if k.endswith(str(sz))}), flush=True)
res["torch_threads_end"] = torch.get_num_threads(); res["host_loadavg"] = os.getloadavg()
print(json.dumps(res, indent=1), flush=True)
