import argparse
import math
import re
from pathlib import Path
from statistics import mean


FLOAT_RE = re.compile(r"[-+]?\d+(?:\.\d+)?")
AZ_RE = re.compile(r"Az=([-+]?\d+(?:\.\d+)?)")
EL_RE = re.compile(r"El=([-+]?\d+(?:\.\d+)?)")
CMD_ID_RE = re.compile(r"cmd_id=(\d+)")
INTERVAL_RE = re.compile(r"interval=([0-9]+(?:\.[0-9]+)?)s")
MASTER_ID_RE = re.compile(r"ID:\s*(\d+)")


def read_text_auto(path: Path) -> str:
    for enc in ("utf-8", "gbk", "cp936"):
        try:
            return path.read_text(encoding=enc)
        except UnicodeDecodeError:
            continue
    return path.read_text(encoding="utf-8", errors="ignore")


def angular_diff(target: float, source: float) -> float:
    return (target - source + 180.0) % 360.0 - 180.0


def ui_to_ctrl_angles(ui_az: float, ui_el: float) -> tuple[float, float]:
    rel_az = ui_az
    if rel_az > 180.0:
        rel_az -= 360.0
    ctrl_az = 90.0 + rel_az
    ctrl_el = ui_el
    if ctrl_az < 0.0:
        ctrl_az = 0.0
    if ctrl_az > 350.0:
        ctrl_az = 350.0
    return ctrl_az, ctrl_el


def percentile(sorted_vals: list[float], p: float) -> float:
    if not sorted_vals:
        return math.nan
    idx = int(max(0, min(len(sorted_vals) - 1, round((len(sorted_vals) - 1) * p))))
    return sorted_vals[idx]


def fmt(x: float) -> str:
    if x != x:  # NaN
        return "n/a"
    return f"{x:.4f}"


def analyze(log_path: Path) -> None:
    text = read_text_auto(log_path)
    lines = text.splitlines()

    phase1 = 0
    phase2 = 0
    phase3 = 0
    laser_count = 0

    err_lines: list[str] = []
    command_az: list[float] = []
    command_el: list[float] = []
    direction_az_err: list[float] = []
    direction_el_err: list[float] = []
    speed_vals: list[float] = []
    laser_ids: list[int] = []
    laser_intervals: list[float] = []
    lock_ids: list[int] = []

    for line in lines:
        if "[Phase 1:" in line:
            phase1 += 1
        if "[Phase 2:" in line:
            phase2 += 1
            m = MASTER_ID_RE.search(line)
            if m:
                lock_ids.append(int(m.group(1)))
        if "[Phase 3:" in line:
            phase3 += 1
            nums = FLOAT_RE.findall(line)
            if nums:
                # Line format contains many numbers; speed is the last one on this line.
                speed_vals.append(float(nums[-1]))

        if "[Laser] Triggered" in line:
            laser_count += 1
            m_id = CMD_ID_RE.search(line)
            if m_id:
                laser_ids.append(int(m_id.group(1)))
            m_dt = INTERVAL_RE.search(line)
            if m_dt:
                laser_intervals.append(float(m_dt.group(1)))

        if ("Err:" in line) or ("[Unexpected]" in line) or ("[Fatal]" in line) or ("Traceback" in line):
            err_lines.append(line)

        if ("->" in line) and ("下发云台指令" in line):
            azs = [float(x) for x in AZ_RE.findall(line)]
            els = [float(x) for x in EL_RE.findall(line)]
            if len(azs) >= 2 and len(els) >= 2:
                fut_az = azs[0]
                fut_el = els[0]
                ctrl_az = azs[-1]
                ctrl_el = els[-1]
                exp_az, exp_el = ui_to_ctrl_angles(fut_az, fut_el)
                direction_az_err.append(abs(angular_diff(ctrl_az, exp_az)))
                direction_el_err.append(abs(ctrl_el - exp_el))
                command_az.append(ctrl_az)
                command_el.append(ctrl_el)

    # Command smoothness stats
    cmd_abs_steps: list[float] = []
    cmd_small_abs_steps: list[float] = []
    sign_flip_count = 0
    tiny_step_count = 0

    if len(command_az) >= 2:
        diffs = [command_az[i] - command_az[i - 1] for i in range(1, len(command_az))]
        cmd_abs_steps = [abs(x) for x in diffs]
        tiny_step_count = sum(1 for x in cmd_abs_steps if x <= 0.05)

        # Exclude large jumps from target switching when checking jitter flips.
        valid_diffs = [d for d in diffs if abs(d) <= 1.0]
        for i in range(1, len(valid_diffs)):
            if (valid_diffs[i] * valid_diffs[i - 1] < 0) and (abs(valid_diffs[i]) > 0.02) and (abs(valid_diffs[i - 1]) > 0.02):
                sign_flip_count += 1
        cmd_small_abs_steps = [abs(d) for d in valid_diffs]

    lock_switches = 0
    for i in range(1, len(lock_ids)):
        if lock_ids[i] != lock_ids[i - 1]:
            lock_switches += 1

    # Print report
    print("=== Tracking Log Analysis ===")
    print(f"log: {log_path}")
    print("")
    print("[Workflow]")
    print(f"phase1_count: {phase1}")
    print(f"phase2_count: {phase2}")
    print(f"phase3_count: {phase3}")
    print(f"laser_trigger_count: {laser_count}")
    print(f"error_count: {len(err_lines)}")
    if err_lines:
        print("first_error:")
        print(err_lines[0])
    print("")
    print("[Direction Consistency]")
    if direction_az_err:
        print(f"checked_commands: {len(direction_az_err)}")
        print(f"az_map_err_mean_deg: {fmt(mean(direction_az_err))}")
        print(f"az_map_err_max_deg: {fmt(max(direction_az_err))}")
        print(f"el_map_err_mean_deg: {fmt(mean(direction_el_err))}")
        print(f"el_map_err_max_deg: {fmt(max(direction_el_err))}")
    else:
        print("checked_commands: 0")
    print("")
    print("[Stability / Jitter]")
    if cmd_abs_steps:
        sorted_steps = sorted(cmd_abs_steps)
        print(f"cmd_step_mean_deg: {fmt(mean(cmd_abs_steps))}")
        print(f"cmd_step_p95_deg: {fmt(percentile(sorted_steps, 0.95))}")
        print(f"cmd_step_max_deg: {fmt(max(cmd_abs_steps))}")
        print(f"tiny_step_count(<=0.05deg): {tiny_step_count}")
        print(f"direction_flip_count(|step|<=1deg): {sign_flip_count}")
    else:
        print("cmd_step_stats: n/a")
    print("")
    print("[Scheduling]")
    print(f"lock_events: {len(lock_ids)}")
    print(f"lock_switches: {lock_switches}")
    if lock_ids:
        unique_ids = sorted(set(lock_ids))
        print(f"locked_target_ids: {unique_ids}")
    print("")
    print("[Laser Trigger]")
    if laser_ids:
        expected_seq = list(range(min(laser_ids), max(laser_ids) + 1))
        continuous = laser_ids == expected_seq
        print(f"cmd_id_min: {min(laser_ids)}")
        print(f"cmd_id_max: {max(laser_ids)}")
        print(f"cmd_id_continuous_from_min: {continuous}")
    else:
        print("cmd_id: n/a")
    if laser_intervals:
        s = sorted(laser_intervals)
        print(f"interval_mean_s: {fmt(mean(laser_intervals))}")
        print(f"interval_p95_s: {fmt(percentile(s, 0.95))}")
        print(f"interval_max_s: {fmt(max(laser_intervals))}")
    else:
        print("interval_stats: n/a")


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze main_tracking log quality and stability.")
    parser.add_argument("--log", default="tools/sim_main_check_10hz_after_tune.log", help="Path to main log file")
    args = parser.parse_args()

    log_path = Path(args.log)
    if not log_path.exists():
        raise FileNotFoundError(f"log not found: {log_path}")

    analyze(log_path)


if __name__ == "__main__":
    main()
