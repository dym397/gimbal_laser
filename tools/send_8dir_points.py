import argparse
import json
import socket
import time

IMG_W = 3840.0
IMG_H = 2160.0


def make_box(cx: float, cy: float, w: float, h: float):
    x1 = max(0.0, cx - w / 2.0)
    y1 = max(0.0, cy - h / 2.0)
    x2 = min(IMG_W - 1.0, cx + w / 2.0)
    y2 = min(IMG_H - 1.0, cy + h / 2.0)
    return [x1, y1, x2, y2]


def build_points():
    cx = IMG_W / 2.0
    cy = IMG_H / 2.0
    x_left = IMG_W * 0.20
    x_right = IMG_W * 0.80
    y_up = IMG_H * 0.20
    y_down = IMG_H * 0.80
    # 顺序: 上、下、左、右、左上、右上、左下、右下
    return [
        ("UP", cx, y_up),
        ("DOWN", cx, y_down),
        ("LEFT", x_left, cy),
        ("RIGHT", x_right, cy),
        ("LEFT_UP", x_left, y_up),
        ("RIGHT_UP", x_right, y_up),
        ("LEFT_DOWN", x_left, y_down),
        ("RIGHT_DOWN", x_right, y_down),
    ]


def run(host: str, port: int, board: str, cam: int, fps: float, hold_s: float, dist_m: float, box_w: float, box_h: float, loops: int):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addr = (host, port)
    dt = 1.0 / fps
    points = build_points()

    print(
        f"[8DIR] sender start -> {host}:{port}, board={board}, cam={cam}, "
        f"fps={fps}, hold_s={hold_s}, dist={dist_m}, loops={loops}"
    )
    print("[8DIR] order: UP, DOWN, LEFT, RIGHT, LEFT_UP, RIGHT_UP, LEFT_DOWN, RIGHT_DOWN")

    for loop_idx in range(loops):
        print(f"[8DIR] loop {loop_idx + 1}/{loops}")
        for name, cx, cy in points:
            box = make_box(cx, cy, box_w, box_h)
            frames = max(1, int(round(hold_s * fps)))
            print(f"[8DIR] sending {name:10s} box={box}")
            for _ in range(frames):
                obj = [box[0], box[1], box[2], box[3], dist_m]
                pkg = {"board": board, "cam": cam, "type": "data", "objs": [obj]}
                sock.sendto(json.dumps(pkg).encode("utf-8"), addr)
                time.sleep(dt)

    sock.close()
    print("[8DIR] done")


def main():
    parser = argparse.ArgumentParser(description="Send 8-direction test targets to main_tracking_v9.py")
    parser.add_argument("--host", default="127.0.0.1", help="receiver host (main_tracking_v9)")
    parser.add_argument("--port", type=int, default=8888, help="receiver UDP port")
    parser.add_argument("--board", default="BOARD_1", help="board field")
    parser.add_argument("--cam", type=int, default=0, help="cam field")
    parser.add_argument("--fps", type=float, default=10.0, help="packet rate")
    parser.add_argument("--hold", type=float, default=2.0, help="seconds per direction")
    parser.add_argument("--dist", type=float, default=300.0, help="mono distance in meters")
    parser.add_argument("--box-w", type=float, default=80.0, help="box width")
    parser.add_argument("--box-h", type=float, default=80.0, help="box height")
    parser.add_argument("--loops", type=int, default=1, help="repeat count of 8-direction sequence")
    args = parser.parse_args()

    run(
        host=args.host,
        port=args.port,
        board=args.board,
        cam=args.cam,
        fps=args.fps,
        hold_s=args.hold,
        dist_m=args.dist,
        box_w=args.box_w,
        box_h=args.box_h,
        loops=args.loops,
    )


if __name__ == "__main__":
    main()

