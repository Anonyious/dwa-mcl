#!/usr/bin/env python3
"""Render an animated GIF from an evaluation trace.

The trace is produced by the evaluation harness itself:

    mcl_evaluate --map map/map.yaml --mode track --trace run.csv
    python tools/render_gif.py run.csv --map map/map.pgm -o data/particleFilter.gif

So the animation shows this package's actual filter output against known
ground truth, not a separate reimplementation.

Needs Pillow (`pip install pillow`); no ROS and no display required.
"""

import argparse

from PIL import Image, ImageDraw

BACKGROUND = (252, 252, 250)
WALL = (38, 38, 42)
UNKNOWN = (226, 226, 230)
PARTICLE = (120, 160, 220)
TRUTH = (0, 168, 89)
ESTIMATE = (214, 58, 44)
TRUTH_PATH = (0, 120, 64)
TEXT = (70, 70, 78)

FREE_PX, OCCUPIED_PX, UNKNOWN_PX = 254, 0, 205


def parse_trace(path):
    meta = {}
    steps = {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#"):
                parts = line.lstrip("# ").split(",")
                meta[parts[0]] = parts[1:]
                continue
            if line.startswith("kind,"):
                continue
            c = line.split(",")
            kind, step = c[0], int(c[1])
            entry = steps.setdefault(step, {"particles": []})
            if kind == "pose":
                entry["truth"] = (float(c[2]), float(c[3]), float(c[4]))
                entry["estimate"] = (float(c[5]), float(c[6]), float(c[7]))
                entry["error"] = float(c[8])
                entry["neff"] = float(c[9]) if c[9] else 0.0
            elif kind == "particle":
                entry["particles"].append((float(c[2]), float(c[3])))
    return meta, steps


def load_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    tokens, i = [], 0
    while len(tokens) < 4:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while i < len(data) and data[i] != 0x0A:
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    i += 1
    width, height = int(tokens[1]), int(tokens[2])
    return width, height, data[i:i + width * height]


def map_image(pgm_path, resolution, size):
    """Map as an RGB image scaled to `size`, plus the world->pixel transform."""
    width, height, pixels = load_pgm(pgm_path)
    img = Image.new("RGB", (width, height))
    put = img.load()
    for y in range(height):
        row = y * width
        for x in range(width):
            v = pixels[row + x]
            if v == OCCUPIED_PX:
                put[x, y] = WALL
            elif v == FREE_PX:
                put[x, y] = BACKGROUND
            else:
                put[x, y] = UNKNOWN
    scale = size / max(width, height)
    out = img.resize((int(width * scale), int(height * scale)), Image.NEAREST)
    # PGM row 0 is the highest y, which is already the image's top row, so the
    # world->pixel map just flips y.
    pixels_per_metre = scale / resolution
    return out, pixels_per_metre, out.size


def render(trace_path, pgm_path, out_path, size=620, stride=3, fps=20, tail=40):
    meta, steps = parse_trace(trace_path)
    resolution = float(meta.get("resolution", [0.05])[0])

    base, ppm, (iw, ih) = map_image(pgm_path, resolution, size)

    def px(x, y):
        return int(round(x * ppm)), int(round(ih - y * ppm))

    order = sorted(steps)
    frames = []
    travelled = []

    for idx in range(0, len(order), stride):
        step = order[idx]
        s = steps[step]
        if "truth" not in s:
            continue
        travelled.append(px(s["truth"][0], s["truth"][1]))

        frame = base.copy()
        d = ImageDraw.Draw(frame)

        for p in s["particles"]:
            cx, cy = px(p[0], p[1])
            d.ellipse([cx - 1, cy - 1, cx + 1, cy + 1], fill=PARTICLE)

        if len(travelled) > 1:
            d.line(travelled, fill=TRUTH_PATH, width=2)

        tx, ty = px(s["truth"][0], s["truth"][1])
        d.ellipse([tx - 5, ty - 5, tx + 5, ty + 5], outline=TRUTH, width=2)

        ex, ey = px(s["estimate"][0], s["estimate"][1])
        d.line([(ex - 5, ey - 5), (ex + 5, ey + 5)], fill=ESTIMATE, width=2)
        d.line([(ex - 5, ey + 5), (ex + 5, ey - 5)], fill=ESTIMATE, width=2)

        d.text((10, 8),
               f'step {step}   error {s["error"]:.2f} m   Neff {s["neff"]:.0f}',
               fill=TEXT)
        d.text((10, 22), "green = ground truth    red = estimate    blue = particles",
               fill=TEXT)
        frames.append(frame)

    if not frames:
        raise SystemExit(f"{trace_path}: no pose rows found")
    frames.extend([frames[-1]] * max(1, tail // max(1, stride)))

    frames[0].save(out_path, save_all=True, append_images=frames[1:],
                   duration=int(1000 / fps), loop=0, optimize=True)
    print(f"{out_path}: {len(frames)} frames, {frames[0].size[0]}x{frames[0].size[1]}, "
          f"from {len(order)} filter updates")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("trace")
    ap.add_argument("--map", required=True, help="path to map.pgm")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--size", type=int, default=620)
    ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--fps", type=int, default=20)
    args = ap.parse_args()
    render(args.trace, args.map, args.out, size=args.size, stride=args.stride,
           fps=args.fps)


if __name__ == "__main__":
    main()
