#!/usr/bin/env python3
"""Object detector server for the web app: Grounding DINO (open vocabulary) on the GPU.

Runs in WSL on the lab PC, in the conda env that has torch + transformers (`detect`). The web app starts it
automatically on the first object search; to start it by hand:  bash detector/start_detector.sh detect

  GET  /ping                                          {"ok": true, "model": ..., "device": ...}
  POST /detect  {"image_png": base64, "prompt": "red mug", "threshold": 0.35}
       -> {"ok": true, "detections": [{"label", "score", "box": [x0, y0, x1, y1]}], "ms": ...}

Listens on 127.0.0.1 only (WSL forwards it to Windows' localhost).
"""
import argparse
import base64
import io
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import torch
from PIL import Image
from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

MODEL = "IDEA-Research/grounding-dino-tiny"
MIN_SIDE = 480  # small frames (the stock 250x250 preview) are upscaled first; the model does better on them


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8770)
    ap.add_argument("--bind", default="127.0.0.1")
    args = ap.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    proc = AutoProcessor.from_pretrained(MODEL)
    model = AutoModelForZeroShotObjectDetection.from_pretrained(MODEL).to(device).eval()
    lock = threading.Lock()

    def detect(img, prompt, threshold):
        scale = 1.0
        if min(img.size) < MIN_SIDE:
            scale = MIN_SIDE / min(img.size)
            img = img.resize((round(img.width * scale), round(img.height * scale)), Image.BICUBIC)
        text = prompt.strip().lower().rstrip(".") + "."
        with lock, torch.no_grad():
            inputs = proc(images=img, text=text, return_tensors="pt").to(device)
            out = model(**inputs)
            det = proc.post_process_grounded_object_detection(
                out, inputs.input_ids, threshold=threshold, text_threshold=0.25, target_sizes=[img.size[::-1]]
            )[0]
        labels = det.get("text_labels", det.get("labels"))
        return [
            {"label": str(lab) or prompt, "score": round(float(s), 3), "box": [round(v / scale, 1) for v in box]}
            for box, s, lab in zip(det["boxes"].tolist(), det["scores"].tolist(), labels)
        ]

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *a):
            print(time.strftime("%H:%M:%S"), fmt % a, flush=True)

        def reply(self, status, obj):
            data = json.dumps(obj).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path.rstrip("/") == "/ping":
                return self.reply(200, {"ok": True, "model": MODEL, "device": device})
            self.reply(404, {"ok": False, "error": "not found"})

        def do_POST(self):
            if self.path.rstrip("/") != "/detect":
                return self.reply(404, {"ok": False, "error": "not found"})
            try:
                n = int(self.headers.get("Content-Length") or 0)
                if n > 20_000_000:
                    return self.reply(413, {"ok": False, "error": "image too large"})
                body = json.loads(self.rfile.read(n))
                prompt = str(body.get("prompt", "")).strip()
                if not prompt:
                    return self.reply(400, {"ok": False, "error": "prompt is empty"})
                threshold = float(body.get("threshold", 0.35))
                img = Image.open(io.BytesIO(base64.b64decode(body["image_png"]))).convert("RGB")
                t = time.monotonic()
                dets = detect(img, prompt, threshold)
                self.reply(200, {"ok": True, "detections": dets, "ms": round((time.monotonic() - t) * 1000)})
            except Exception as e:  # keep serving
                print("error:", repr(e), flush=True)
                self.reply(500, {"ok": False, "error": repr(e)})

    server = ThreadingHTTPServer((args.bind, args.port), Handler)
    print(f"detector ready on {args.bind}:{args.port} ({MODEL}, {device})", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
