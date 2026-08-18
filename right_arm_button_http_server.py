#!/usr/bin/env python3
"""Temporary synchronous HTTP adapter for the RM75 right-arm button test."""
import argparse
import json
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


class State:
    def __init__(self, workspace):
        self.workspace = Path(workspace).expanduser().resolve()
        self.task_lock = threading.Lock()
        self.data_lock = threading.Lock()
        self.data = {"task_id": "", "status": "idle", "type": "detection",
                     "detection_type": "all", "message": "Ready"}

    def snapshot(self):
        with self.data_lock:
            return dict(self.data)

    def set(self, **values):
        with self.data_lock:
            self.data.update(values)

    def execute(self, task_id):
        if not self.task_lock.acquire(False):
            return 409, {"task_id": task_id, "status": "failed",
                         "type": "detection", "detection_type": "all",
                         "message": "Another task is already executing"}
        task_dir = self.workspace / "right_button_service" / "task_logs" / task_id
        task_dir.mkdir(parents=True, exist_ok=True)
        started = time.time()
        self.set(task_id=task_id, status="executing", message="Task is executing",
                 started_at=started, finished_at=0.0)
        try:
            with (task_dir / "terminal.log").open("w", encoding="utf-8") as log:
                proc = subprocess.run(
                    [str(self.workspace / "right_arm_press_button.sh"),
                     "run-task", task_id],
                    stdout=log, stderr=subprocess.STDOUT, timeout=360, check=False)
            ok = proc.returncode == 0
            code = 200 if ok else 500
            status = "success" if ok else "failed"
            message = ("Task completed successfully" if ok else
                       f"Right-arm button task failed (exit={proc.returncode})")
        except subprocess.TimeoutExpired:
            code, status, message = 504, "failed", "Right-arm button task timed out"
        except Exception as exc:
            code, status, message = 500, "failed", f"Right-arm task error: {exc}"
        finally:
            finished = time.time()
            result = {"task_id": task_id, "status": status, "type": "detection",
                      "detection_type": "all", "message": message,
                      "started_at": started, "finished_at": finished}
            self.set(**result)
            (task_dir / "task_result.json").write_text(
                json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
            self.task_lock.release()
        return code, result


def handler_for(state):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, code, payload):
            body = json.dumps(payload, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path.rstrip("/") == "/health":
                snap = state.snapshot()
                self.reply(200, {"ok": True, "ready": snap["status"] != "executing",
                                 "service": "right_arm_button_test",
                                 "start": True, "status": True, "task": snap})
            elif parsed.path.rstrip("/") == "/status":
                wanted = parse_qs(parsed.query).get("task_id", [""])[0]
                snap = state.snapshot()
                if wanted and snap["task_id"] and wanted != snap["task_id"]:
                    self.reply(404, {"status": "failed", "message": "task_id not found"})
                else:
                    self.reply(200, snap)
            else:
                self.reply(404, {"status": "failed", "message": "Not found"})

        def do_POST(self):
            if urlparse(self.path).path.rstrip("/") != "/start_task":
                self.reply(404, {"status": "failed", "message": "Not found"}); return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                data = json.loads(self.rfile.read(size).decode())
            except Exception as exc:
                self.reply(400, {"status": "failed", "message": f"Invalid JSON: {exc}"}); return
            task_type = str(data.get("task_type", "")).strip()
            task_id = str(data.get("task_id", "")).strip()
            if task_type != "detection":
                self.reply(400, {"task_id": task_id, "status": "failed",
                                 "type": task_type,
                                 "message": "This test service only handles detection"}); return
            if not task_id:
                self.reply(400, {"status": "failed", "type": "detection",
                                 "message": "task_id is required"}); return
            code, result = state.execute(task_id)
            self.reply(code, result)
    return Handler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--workspace", default="~/ros2_ws")
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), handler_for(State(args.workspace)))
    print(f"Right-arm test service: http://{args.host}:{args.port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
