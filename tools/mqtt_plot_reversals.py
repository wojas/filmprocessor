#!/usr/bin/env python3
import argparse
from io import StringIO
import sys
import threading
import queue
from collections import deque

import paho.mqtt.client as mqtt
import pandas as pd
import numpy as np

# ------------ CSV parser ------------
def parse_csv_payload(text: str) -> pd.DataFrame:
    print("Parsing CSV payload")
    lines = [ ln.replace('#ts', 'ts') for ln in text.splitlines() if ln.strip() ]
    data = "\n".join(lines)
    df = pd.read_csv(StringIO(data))
    required = {"ts", "dir", "rpm", "duty"}
    if not required.issubset(df.columns):
        missing = required - set(df.columns)
        raise ValueError(f"Missing required columns in CSV: {missing}")
    for c in df.columns:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    df = df.dropna(subset=["ts"]).reset_index(drop=True)
    return df

# ------------ Transition detection ------------
def find_transitions_clean(df: pd.DataFrame, lookahead_rows: int = 6, max_gap_ms: float = 300.0) -> np.ndarray:
    if len(df) < 2:
        return np.array([], dtype=int)
    ts = pd.to_numeric(df["ts"], errors="coerce").values
    ddir = pd.to_numeric(df["dir"], errors="coerce").fillna(-999).astype(int).values
    idx = []

    prev = np.roll(ddir, 1)
    direct = np.where((prev == -1) & (ddir == 1))[0]
    idx.extend(direct.tolist())

    for i in range(1, len(ddir)):
        if ddir[i-1] == -1 and ddir[i] == 0:
            for j in range(i+1, min(i+1+lookahead_rows, len(ddir))):
                if ddir[j] == 1 and (ts[j] - ts[i]) <= max_gap_ms:
                    idx.append(j)
                    break
    return np.array(sorted(set(idx)), dtype=int)

def has_recent_negative_dir(df: pd.DataFrame, t_rev: float, window_ms: float = 300.0) -> bool:
    sub = df[(df["ts"] >= t_rev - window_ms) & (df["ts"] < t_rev)]
    if sub.empty:
        return False
    vals = pd.to_numeric(sub["dir"], errors="coerce").dropna().astype(int)
    return (vals == -1).any()

def extract_window(df: pd.DataFrame, t_rev: float, window_ms: int):
    t0 = t_rev - 500.0
    t1 = t0 + window_ms
    sub = df[(df["ts"] >= t0) & (df["ts"] <= t1)].copy()
    if sub.empty:
        return None
    sub = sub.sort_values("ts")
    t_rel = sub["ts"].to_numpy(dtype=float) - t0
    t_rel = t_rel - (t_rel.min() if len(t_rel) else 0.0)
    t_rel = np.clip(t_rel, 0.0, float(window_ms))
    rpm = pd.to_numeric(sub["rpm"], errors="coerce").to_numpy(dtype=float)
    duty = pd.to_numeric(sub["duty"], errors="coerce").to_numpy(dtype=float)
    return np.vstack([t_rel, rpm, duty])

# ------------ MQTT Runner ------------
class Runner:
    def __init__(self, args):
        self.args = args
        self.topic = args.topic
        self.window_ms = args.window_ms
        self.buffer_ms = args.buffer_ms

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=args.client_id or "")
        if args.username:
            self.client.username_pw_set(args.username, args.password or None)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.history = pd.DataFrame(columns=["ts","dir","rpm","duty"])
        self.pending = {}
        self.seen = set()
        self.ready = queue.Queue()
        self.lock = threading.Lock()

    def vlog(self, *a):
        if self.args.verbose:
            print("[DBG]", *a, flush=True)

    def on_connect(self, client, userdata, flags, rc, props=None):
        print(f"Connected to MQTT {self.args.host}:{self.args.port}; subscribing to '{self.topic}'")
        client.subscribe(self.topic, qos=0)

    def on_message(self, client, userdata, msg):
        payload = msg.payload.decode("utf-8", errors="ignore")
        try:
            df = parse_csv_payload(payload)
            if df.empty:
                return
            self.vlog(f"packet: rows={len(df)} ts=[{df['ts'].min():.1f},{df['ts'].max():.1f}] dir_unique={sorted(pd.to_numeric(df['dir'], errors='coerce').dropna().astype(int).unique().tolist())}")
            self._merge(df)
            self._discover()
            self._enqueue_ready()
        except Exception as e:
            print("Error:", e, file=sys.stderr)

    def _merge(self, df_new):
        with self.lock:
            self.history = pd.concat([self.history, df_new], ignore_index=True)
            self.history = self.history.drop_duplicates(subset=["ts"], keep="last").sort_values("ts").reset_index(drop=True)
            if not self.history.empty:
                tmax = float(self.history["ts"].max())
                self.history = self.history[self.history["ts"] >= tmax - self.buffer_ms].reset_index(drop=True)
                self.vlog(f"history: rows={len(self.history)} ts=[{self.history['ts'].min():.1f},{self.history['ts'].max():.1f}] (kept {self.buffer_ms} ms)")

    def _discover(self):
        with self.lock:
            df = self.history.copy()
        if df.empty:
            return
        idx = find_transitions_clean(df)
        self.vlog(f"transitions found: {len(idx)}")
        for i in idx:
            t_rev = float(df.iloc[i]["ts"])
            key = round(t_rev / 10.0)
            if key in self.seen or t_rev in self.pending:
                continue
            if not has_recent_negative_dir(df, t_rev, window_ms=300.0):
                self.vlog(f"skip t_rev={t_rev:.1f} (no recent -1)")
                continue
            t0 = t_rev - 500.0
            t1 = t0 + self.window_ms
            self.pending[t_rev] = (t0, t1)
            self.vlog(f"pending window for t_rev={t_rev:.1f}: [{t0:.1f},{t1:.1f}]")

    def _enqueue_ready(self):
        with self.lock:
            df = self.history.copy()
        if df.empty:
            return
        have_max = float(df["ts"].max())
        for t_rev, (t0, t1) in list(self.pending.items()):
            if have_max >= t1:
                arr = extract_window(df, t_rev, self.window_ms)
                if arr is not None and arr.shape[1] >= 5:
                    self.vlog(f"READY: enqueue window t_rev={t_rev:.1f} samples={arr.shape[1]}")
                    self.ready.put((t_rev, arr))
                    self.seen.add(round(t_rev / 10.0))
                self.pending.pop(t_rev, None)

    def poll_ready(self, max_items=20):
        out = []
        try:
            for _ in range(max_items):
                out.append(self.ready.get_nowait())
        except queue.Empty:
            pass
        return out

    def start(self):
        self.client.connect(self.args.host, self.args.port, keepalive=60)
        self.client.loop_start()

    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()

# ------------ Main (plotting) ------------
def main():
    p = argparse.ArgumentParser(description="MQTT reversal plotter")
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--username", default=None)
    p.add_argument("--password", default=None)
    p.add_argument("--client-id", default=None)
    p.add_argument("--topic", default="letsroll/motor/csv")
    p.add_argument("--window-ms", type=int, default=6_000)
    p.add_argument("--buffer-ms", type=int, default=60_000)
    p.add_argument("--refresh-ms", type=int, default=1_000)
    p.add_argument("--backend", default=None)
    p.add_argument("--verbose", action="store_true")
    args = p.parse_args()

    import matplotlib
    if args.backend:
        matplotlib.use(args.backend, force=True)
    else:
        if sys.platform == "darwin":
            try:
                matplotlib.use("MacOSX", force=True)
            except Exception:
                matplotlib.use("TkAgg", force=True)

    import matplotlib.pyplot as plt

    class LivePlotter:
        def __init__(self, window_ms):
            self.window_ms = window_ms
            self.fig, self.ax_rpm = plt.subplots(figsize=(10,5))
            self.ax_duty = self.ax_rpm.twinx()
            self._init()

        def _init(self):
            self.ax_rpm.set_title("Reversal-aligned periods (keep last 10) — 0 ms = 500 ms pre-reversal")
            self.ax_rpm.set_xlabel("Time since window start (ms)")
            self.ax_rpm.set_ylabel("RPM", color="green", fontweight="bold")
            self.ax_duty.set_ylabel("Duty", color="red", fontweight="bold")
            self.ax_rpm.tick_params(axis='y', colors='green')
            self.ax_duty.tick_params(axis='y', colors='red')
            self.ax_rpm.grid(True)
            self.ax_rpm.set_xlim(0, self.window_ms)
            self.ax_rpm.axvline(500, linestyle="--", linewidth=1, color="gray")

        def redraw(self, windows: deque):
            self.ax_rpm.cla()
            self.ax_duty.cla()
            self._init()
            rpm_lines = []
            duty_lines = []
            for (_trev, arr) in windows:
                t_rel, rpm, duty = arr
                rpm_line, = self.ax_rpm.plot(t_rel, rpm, color="green", linewidth=1.6, alpha=0.9, label="RPM")
                duty_line, = self.ax_duty.plot(t_rel, duty, color="red", linewidth=1.0, alpha=0.8, label="Duty")
                rpm_lines.append(rpm_line)
                duty_lines.append(duty_line)
            if rpm_lines or duty_lines:
                labels = ["RPM", "Duty"]
                handles = [rpm_lines[-1] if rpm_lines else None, duty_lines[-1] if duty_lines else None]
                handles = [h for h in handles if h]
                self.ax_rpm.legend(handles, labels, loc="upper right")
            self.fig.canvas.draw_idle()

    runner = Runner(args)
    plotter = LivePlotter(args.window_ms)
    runner.start()

    windows = deque(maxlen=10)

    def on_timer():
        new = runner.poll_ready()
        if new and args.verbose:
            print(f"[DBG] add {len(new)} window(s) -> buffer={len(windows)+len(new)}", flush=True)
        if new:
            windows.extend(new)
        plotter.redraw(windows)

    timer = plotter.fig.canvas.new_timer(interval=args.refresh_ms)
    timer.add_callback(on_timer)
    timer.start()

    try:
        plt.show()
    finally:
        runner.stop()

if __name__ == "__main__":
    main()
