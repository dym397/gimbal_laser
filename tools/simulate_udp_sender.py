import argparse
import json
import random
import socket
import time
from dataclasses import dataclass

IMG_W = 3840.0
IMG_H = 2160.0


@dataclass
class SimTarget:
    x: float
    y: float
    vx: float
    vy: float
    w: float
    h: float
    ax: float = 0.0
    ay: float = 0.0

    def step(self):
        self.vx += self.ax
        self.vy += self.ay
        self.x += self.vx
        self.y += self.vy

    def box(self):
        return [self.x, self.y, self.x + self.w, self.y + self.h]


def send_pkg(sock, addr, board, cam, objs):
    pkg = {"board": board, "cam": cam, "objs": objs}
    sock.sendto(json.dumps(pkg).encode("utf-8"), addr)


def run(host: str, port: int, fps: float, seconds: float, seed: int):
    rng = random.Random(seed)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (host, port)
    dt = 1.0 / fps
    frames = int(seconds * fps)

    t_a = SimTarget(x=3700.0, y=1035.0, vx=6.0, vy=0.0, w=60.0, h=90.0)
    t_b = SimTarget(x=1885.0, y=990.0, vx=0.0, vy=2.1, w=50.0, h=150.0)
    t_c = SimTarget(x=420.0, y=1230.0, vx=-2.8, vy=0.0, w=45.0, h=130.0)

    f_warm = int(frames * 0.15)
    f_cross = int(frames * 0.35)
    f_outlier = int(frames * 0.55)
    f_gap = int(frames * 0.72)
    f_recover = int(frames * 0.88)

    print(f"[SIM] start -> host={host}:{port}, fps={fps}, seconds={seconds}, frames={frames}, seed={seed}")

    for k in range(frames):
        if k == f_warm:
            print("[SIM] phase -> crossing / acceleration")
            t_a.vx, t_a.vy = 8.0, 0.6
            t_b.vx, t_b.vy = 0.8, 2.8
            t_c.vx, t_c.vy = -3.6, 0.2

        if f_warm <= k < f_cross:
            t_a.ax, t_b.ax, t_c.ax = 0.02, -0.01, 0.01
        else:
            t_a.ax = t_b.ax = t_c.ax = 0.0

        inject_outlier = (f_cross <= k < f_outlier) and ((k - f_cross) % 12 == 0)

        if f_outlier <= k < f_gap and (k % 3 == 0):
            time.sleep(dt)
            continue

        if k == f_gap:
            print("[SIM] phase -> recovery")
            t_a.vx, t_a.vy = 3.0, 0.0
            t_b.vx, t_b.vy = 0.3, 1.2
            t_c.vx, t_c.vy = -1.2, 0.0

        drop_c = (k >= f_recover) and (k % 10 < 4)

        t_a.step()
        t_b.step()
        t_c.step()

        t_a.x += rng.uniform(-0.5, 0.5)
        t_b.y += rng.uniform(-0.7, 0.7)
        t_c.x += rng.uniform(-0.6, 0.6)

        objs = [t_a.box(), t_b.box()]
        if not drop_c:
            objs.append(t_c.box())

        if inject_outlier:
            objs = [
                [-220.0, 1020.0, -160.0, 1110.0],
                [1920.0, -220.0, 1970.0, -130.0],
                [4040.0, 1240.0, 4120.0, 1370.0],
            ]

        if k % 30 == 0 and not inject_outlier:
            stale = [[500.0, 500.0, 560.0, 620.0]]
            send_pkg(sock, addr, "BOARD_1", 0, stale)

        send_pkg(sock, addr, "BOARD_1", 0, objs)
        time.sleep(dt)

    sock.close()
    print("[SIM] done")


def main():
    parser = argparse.ArgumentParser(description="Comprehensive UDP simulator for main_tracking_v9.py")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8888)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--seconds", type=float, default=14.0)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    run(args.host, args.port, args.fps, args.seconds, args.seed)


if __name__ == "__main__":
    main()
