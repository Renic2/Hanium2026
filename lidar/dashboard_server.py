#!/usr/bin/env python3
"""Serve the dashboard and lightweight RDK-X5 host resource telemetry."""

import glob
import json
import os
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit


class ResourceSampler:
    def __init__(self):
        self._lock = threading.Lock()
        self._last_cpu = self._read_cpu_ticks()

    @staticmethod
    def _read_cpu_ticks():
        with open("/proc/stat", "r", encoding="ascii") as handle:
            fields = handle.readline().split()[1:]
        values = [int(value) for value in fields]
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        return sum(values), idle

    @staticmethod
    def _memory():
        values = {}
        with open("/proc/meminfo", "r", encoding="ascii") as handle:
            for line in handle:
                key, raw_value = line.split(":", 1)
                values[key] = int(raw_value.split()[0]) * 1024
        total = values["MemTotal"]
        available = values.get("MemAvailable", values.get("MemFree", 0))
        used = max(0, total - available)
        return total, used

    @staticmethod
    def _temperature_c():
        candidates = []
        paths = glob.glob("/sys/class/thermal/thermal_zone*/temp")
        paths += glob.glob("/sys/class/hwmon/hwmon*/temp*_input")
        for path in paths:
            try:
                with open(path, "r", encoding="ascii") as handle:
                    value = float(handle.read().strip())
                if value > 1000.0:
                    value /= 1000.0
                if 0.0 < value < 130.0:
                    candidates.append(value)
            except (OSError, ValueError):
                continue
        return round(max(candidates), 1) if candidates else None

    def sample(self):
        with self._lock:
            current_cpu = self._read_cpu_ticks()
            previous_total, previous_idle = self._last_cpu
            self._last_cpu = current_cpu
        total_delta = current_cpu[0] - previous_total
        idle_delta = current_cpu[1] - previous_idle
        cpu_percent = None
        if total_delta > 0:
            cpu_percent = 100.0 * (1.0 - idle_delta / total_delta)
        memory_total, memory_used = self._memory()
        load_1m, load_5m, load_15m = os.getloadavg()
        with open("/proc/uptime", "r", encoding="ascii") as handle:
            uptime_sec = float(handle.read().split()[0])
        return {
            "cpu_percent": round(cpu_percent, 1) if cpu_percent is not None else None,
            "cpu_count": os.cpu_count(),
            "memory_total_bytes": memory_total,
            "memory_used_bytes": memory_used,
            "memory_percent": round(100.0 * memory_used / memory_total, 1),
            "load_1m": round(load_1m, 2),
            "load_5m": round(load_5m, 2),
            "load_15m": round(load_15m, 2),
            "temperature_c": self._temperature_c(),
            "uptime_sec": round(uptime_sec),
        }


RESOURCE_SAMPLER = ResourceSampler()


class DashboardHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory="/dashboard", **kwargs)

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/health":
            body = b"ok"
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        if path == "/api/resources":
            body = json.dumps(RESOURCE_SAMPLER.sample()).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return
        super().do_GET()

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        super().end_headers()

    def log_message(self, _format, *_args):
        pass


if __name__ == "__main__":
    server = ThreadingHTTPServer(("127.0.0.1", 8080), DashboardHandler)
    print("[dashboard] listening on http://127.0.0.1:8080", flush=True)
    server.serve_forever()
