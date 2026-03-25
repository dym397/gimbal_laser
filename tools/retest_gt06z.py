import argparse
import json
import os
import socket
import struct
import subprocess
import threading
import time
from collections import defaultdict
from pathlib import Path


PACK_FMT = "!BB8sIfff"
PACK_SIZE = struct.calcsize(PACK_FMT)


def _build_sender_frames(seconds: float, fps: float):
    total = int(seconds * fps)
    dt = 1.0 / fps

    # Three stable targets with mild motion and per-target mono distance.
    x1 = 1180.0
    x2 = 2620.0
    x3 = 1880.0
    y1 = 980.0
    y2 = 1010.0
    y3 = 840.0

    for k in range(total):
        x1 += 0.35
        x2 -= 0.25
        y3 += 0.10

        objs = [
            {"box": [x1, y1, x1 + 72.0, y1 + 128.0], "distance": 430.0},
            {"box": [x2, y2, x2 + 68.0, y2 + 120.0], "distance": 470.0},
            {"box": [x3, y3, x3 + 64.0, y3 + 112.0], "distance": 455.0},
        ]
        yield dt, objs


def _udp_sender(host: str, port: int, board: str, cam: int, seconds: float, fps: float):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (host, port)
    try:
        for dt, objs in _build_sender_frames(seconds, fps):
            pkg = {"board": board, "cam": cam, "objs": objs}
            sock.sendto(json.dumps(pkg).encode("utf-8"), addr)
            time.sleep(dt)
    finally:
        sock.close()


def _ui_listener(bind_ip: str, bind_port: int, stop_flag: dict, records: list):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_ip, bind_port))
    sock.settimeout(0.2)
    try:
        while not stop_flag["stop"]:
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                continue
            if len(data) < PACK_SIZE:
                continue
            msg_type, cam, board, tid, az, el, dist = struct.unpack(PACK_FMT, data[:PACK_SIZE])
            records.append((msg_type, cam, tid, az, el, dist, time.time()))
    finally:
        sock.close()


def _build_main_command(
    gimbal_port: str,
    ui_ip: str,
    ui_port: int,
    local_port: int,
    az_base: float,
):
    # Run main with runtime overrides without modifying source files.
    code = (
        "import main_tracking_v9 as m;"
        f"m.USE_MOCK_GIMBAL=False;"
        f"m.GIMBAL_PORT={gimbal_port!r};"
        f"m.UI_IP={ui_ip!r};"
        f"m.UI_PORT={ui_port};"
        f"m.LOCAL_PORT={local_port};"
        f"m.GIMBAL_AZ_BASE={az_base};"
        "m.main()"
    )
    return ["python", "-c", code]


def main():
    parser = argparse.ArgumentParser(description="Retest main_tracking_v9.py with real GT06Z gimbal.")
    parser.add_argument("--gimbal-port", default="COM3", help="GT06Z serial port, e.g. COM3")
    parser.add_argument("--ui-ip", default="127.0.0.1", help="UI receiver IP")
    parser.add_argument("--ui-port", type=int, default=9999, help="UI receiver port")
    parser.add_argument("--local-port", type=int, default=8888, help="main_tracking UDP input port")
    parser.add_argument("--sender-host", default="127.0.0.1", help="UDP sender target host")
    parser.add_argument("--sender-port", type=int, default=8888, help="UDP sender target port")
    parser.add_argument("--board", default="BOARD_1", help="board field in UDP packets")
    parser.add_argument("--cam", type=int, default=2, help="cam field in UDP packets")
    parser.add_argument("--seconds", type=float, default=30.0, help="test duration in seconds")
    parser.add_argument("--fps", type=float, default=10.0, help="UDP packet rate")
    parser.add_argument("--startup-wait", type=float, default=2.5, help="wait before sending UDP")
    parser.add_argument("--tail-wait", type=float, default=2.0, help="wait after sender done")
    parser.add_argument("--az-base", type=float, default=90.0, help="runtime override for GIMBAL_AZ_BASE")
    parser.add_argument("--listen-ui", action="store_true", help="listen locally and summarize UI packets")
    parser.add_argument("--ui-bind-ip", default="0.0.0.0", help="local bind ip for UI listener")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    logs_dir = root / "tools"
    logs_dir.mkdir(exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    main_log = logs_dir / f"gt06z_main_{ts}.log"

    print(f"[RUN] workspace={root}")
    print(f"[RUN] main_log={main_log}")
    print(f"[RUN] gimbal_port={args.gimbal_port}")
    print(f"[RUN] use_mock=False")
    print(f"[RUN] udp_sender -> {args.sender_host}:{args.sender_port}, fps={args.fps}, seconds={args.seconds}")
    print(f"[RUN] ui_target  -> {args.ui_ip}:{args.ui_port}")

    records = []
    stop_flag = {"stop": False}
    listener_thread = None
    if args.listen_ui:
        listener_thread = threading.Thread(
            target=_ui_listener,
            args=(args.ui_bind_ip, args.ui_port, stop_flag, records),
            daemon=True,
        )
        listener_thread.start()
        print(f"[RUN] ui_listener ON at {args.ui_bind_ip}:{args.ui_port}")

    cmd = _build_main_command(
        gimbal_port=args.gimbal_port,
        ui_ip=args.ui_ip,
        ui_port=args.ui_port,
        local_port=args.local_port,
        az_base=args.az_base,
    )

    with main_log.open("w", encoding="utf-8", errors="ignore") as f_main:
        proc = subprocess.Popen(cmd, cwd=str(root), stdout=f_main, stderr=subprocess.STDOUT)
        try:
            time.sleep(args.startup_wait)
            _udp_sender(
                host=args.sender_host,
                port=args.sender_port,
                board=args.board,
                cam=args.cam,
                seconds=args.seconds,
                fps=args.fps,
            )
            time.sleep(args.tail_wait)
        finally:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=5)

    stop_flag["stop"] = True
    if listener_thread is not None:
        listener_thread.join(timeout=2)

    print("[DONE] main process stopped")
    if not args.listen_ui:
        print("[INFO] UI listener disabled. Re-run with --listen-ui to collect send statistics.")
        return

    print(f"[UI] packet_count={len(records)}")
    if not records:
        print("[UI] no packets captured")
        return

    by_tid = defaultdict(list)
    for _, _, tid, _, _, dist, _ in records:
        by_tid[tid].append(dist)

    print(f"[UI] target_count={len(by_tid)}")
    for tid in sorted(by_tid):
        vals = by_tid[tid]
        tail = vals[-min(60, len(vals)):]
        dmin = min(tail)
        dmax = max(tail)
        dmean = sum(tail) / len(tail)
        has_525 = any(abs(v - 52.5) < 1e-6 for v in tail)
        print(
            f"[UI] tid={tid} tail_n={len(tail)} "
            f"mean={dmean:.3f} min={dmin:.3f} max={dmax:.3f} has_52.5={has_525}"
        )


if __name__ == "__main__":
    main()
