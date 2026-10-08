#!/usr/bin/env python3
"""Render an animated GIF from a planner trace.

The trace is produced by the planner itself:

    dwa_local_planner --scenario default --no-gui --trace run.csv
    python tools/render_gif.py run.csv -o data/dwa_default.gif

So the animation is this program's actual output, not a separate
reimplementation of the planner.

Needs Pillow (`pip install pillow`); no OpenCV and no display required.
"""

import argparse
import math

from PIL import Image, ImageDraw

BACKGROUND = (252, 252, 250)
WALL = (38, 38, 42)
INFLATION = (222, 222, 228)
GOAL = (0, 168, 89)
GOAL_RING = (0, 120, 64)
PATH = (120, 130, 145)
PLAN = (214, 58, 44)
ROBOT = (28, 28, 32)
RECOVERY = (0, 112, 214)
TEXT = (90, 90, 98)


def parse_trace(path):
    meta = {"obstacles": []}
    poses, plans = [], {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                parts = line.lstrip("# ").split(",")
                key = parts[0]
                if key == "obstacle":
                    meta["obstacles"].append((float(parts[1]), float(parts[2])))
                elif key == "goal":
                    meta["goal"] = (float(parts[1]), float(parts[2]))
                elif key == "scenario":
                    meta["scenario"] = parts[1]
                else:
                    meta[key] = float(parts[1])
                continue
            if line.startswith("kind,"):
                continue
            f_ = line.split(",")
            kind, step = f_[0], int(f_[1])
            if kind == "pose":
                poses.append({
                    "step": step,
                    "x": float(f_[3]), "y": float(f_[4]), "theta": float(f_[5]),
                    "v": float(f_[6]) if f_[6] else 0.0,
                    "w": float(f_[7]) if f_[7] else 0.0,
                    "recovery": f_[8] == "1",
                })
            elif kind == "traj":
                plans.setdefault(step, []).append((float(f_[3]), float(f_[4])))
    return meta, poses, plans


class View:
    """World metres -> image pixels, fitted once to the whole scene."""

    def __init__(self, meta, poses, size, margin_m):
        xs = [p["x"] for p in poses] + [meta["goal"][0]]
        ys = [p["y"] for p in poses] + [meta["goal"][1]]
        for ox, oy in meta["obstacles"]:
            xs.append(ox)
            ys.append(oy)

        pad = margin_m + meta.get("collision_radius", 1.0)
        min_x, max_x = min(xs) - pad, max(xs) + pad
        min_y, max_y = min(ys) - pad, max(ys) + pad

        extent = max(max_x - min_x, max_y - min_y, 1e-6)
        self.size = size
        self.scale = size / extent
        self.origin_x = 0.5 * (min_x + max_x) - 0.5 * extent
        self.origin_y = 0.5 * (min_y + max_y) - 0.5 * extent

    def px(self, x, y):
        return (int(round((x - self.origin_x) * self.scale)),
                int(round(self.size - (y - self.origin_y) * self.scale)))

    def r(self, metres):
        return max(1, int(round(metres * self.scale)))


def draw_static(view, meta):
    img = Image.new("RGB", (view.size, view.size), BACKGROUND)
    d = ImageDraw.Draw(img)

    collision = meta.get("collision_radius", 1.0)
    for ox, oy in meta["obstacles"]:
        cx, cy = view.px(ox, oy)
        rr = view.r(collision)
        d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], fill=INFLATION)
    for ox, oy in meta["obstacles"]:
        cx, cy = view.px(ox, oy)
        rr = view.r(0.35)
        d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], fill=WALL)

    gx, gy = view.px(*meta["goal"])
    tol = view.r(meta.get("goal_tolerance", 1.0))
    d.ellipse([gx - tol, gy - tol, gx + tol, gy + tol], outline=GOAL_RING, width=2)
    rr = view.r(0.35)
    d.ellipse([gx - rr, gy - rr, gx + rr, gy + rr], fill=GOAL)
    return img


def render(trace, out_path, size=560, margin=1.5, stride=4, fps=25, tail=60):
    meta, poses, plans = parse_trace(trace)
    if not poses:
        raise SystemExit(f"{trace}: no pose rows found")

    view = View(meta, poses, size, margin)
    base = draw_static(view, meta)
    robot_r = meta.get("robot_radius", 1.0)

    frames = []
    for i in range(0, len(poses), stride):
        p = poses[i]
        frame = base.copy()
        d = ImageDraw.Draw(frame)

        # Travelled path so far.
        if i:
            pts = [view.px(q["x"], q["y"]) for q in poses[:i + 1]]
            d.line(pts, fill=PATH, width=2)

        # The trajectory the planner actually chose this cycle.
        plan = plans.get(p["step"], [])
        if len(plan) > 1:
            d.line([view.px(x, y) for x, y in plan], fill=PLAN, width=3)

        colour = RECOVERY if p["recovery"] else ROBOT
        cx, cy = view.px(p["x"], p["y"])
        rr = view.r(robot_r)
        d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=colour, width=3)
        hx, hy = view.px(p["x"] + robot_r * math.cos(p["theta"]),
                         p["y"] + robot_r * math.sin(p["theta"]))
        d.line([(cx, cy), (hx, hy)], fill=colour, width=3)

        label = (f'{meta.get("scenario", "")}   step {p["step"]}   '
                 f'v {p["v"]:+.2f} m/s   w {p["w"]:+.2f} rad/s')
        if p["recovery"]:
            label += "   RECOVERY"
        d.text((10, 8), label, fill=RECOVERY if p["recovery"] else TEXT)
        frames.append(frame)

    # Hold the final frame so the loop reads clearly.
    frames.extend([frames[-1]] * max(1, tail // max(1, stride)))

    frames[0].save(out_path, save_all=True, append_images=frames[1:],
                   duration=int(1000 / fps), loop=0, optimize=True)
    print(f"{out_path}: {len(frames)} frames, {size}x{size}, "
          f"from {len(poses)} planner cycles")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--size", type=int, default=560)
    ap.add_argument("--stride", type=int, default=4,
                    help="render every Nth planner cycle")
    ap.add_argument("--fps", type=int, default=25)
    args = ap.parse_args()
    render(args.trace, args.out, size=args.size, stride=args.stride, fps=args.fps)


if __name__ == "__main__":
    main()
