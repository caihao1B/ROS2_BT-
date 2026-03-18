#!/usr/bin/env python3
import argparse
import math
import os
import re
from pathlib import Path

def load_map_yaml(path: Path) -> dict:
    # 优先用 PyYAML；如果没有，做一个极简解析（够用 map.yaml 这种格式）
    try:
        import yaml  # type: ignore
        with path.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f)
    except Exception:
        d = {}
        cur_key = None
        for line in path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if re.match(r"^[A-Za-z_]+\s*:", line):
                k, v = line.split(":", 1)
                k = k.strip()
                v = v.strip()
                if v == "":
                    d[k] = []
                    cur_key = k
                else:
                    # number/bool/string
                    if v.lower() in ("true","false"):
                        d[k] = (v.lower()=="true")
                    else:
                        try:
                            if "." in v:
                                d[k] = float(v)
                            else:
                                d[k] = int(v)
                        except Exception:
                            d[k] = v
                    cur_key = None
            elif line.startswith("-") and cur_key:
                val = line[1:].strip()
                try:
                    d[cur_key].append(float(val))
                except Exception:
                    d[cur_key].append(val)
        return d

def read_pgm(path: Path):
    data = path.read_bytes()
    # PGM binary P5
    if not data.startswith(b"P5"):
        raise RuntimeError(f"Not a binary PGM (P5): {path}")
    i = 2
    def skip_ws():
        nonlocal i
        while i < len(data) and data[i] in b" \t\r\n":
            i += 1
    def read_token():
        nonlocal i
        skip_ws()
        # skip comments
        while i < len(data) and data[i] == ord("#"):
            while i < len(data) and data[i] != ord("\n"):
                i += 1
            skip_ws()
        j = i
        while j < len(data) and data[j] not in b" \t\r\n":
            j += 1
        tok = data[i:j]
        i = j
        return tok

    w = int(read_token())
    h = int(read_token())
    maxval = int(read_token())
    if maxval > 255:
        raise RuntimeError("Only 8-bit PGM supported in this script.")

    # one byte per pixel
    skip_ws()
    pixels = data[i:i+w*h]
    if len(pixels) != w*h:
        raise RuntimeError("PGM pixel data truncated.")
    return w, h, pixels  # pixels is bytes length w*h, row-major from top to bottom

def build_occ_mask(cfg: dict, w: int, h: int, pixels: bytes):
    res = float(cfg.get("resolution", 0.05))
    origin = cfg.get("origin", [0.0, 0.0, 0.0])
    if isinstance(origin, dict):
        origin = [float(origin.get("x",0.0)), float(origin.get("y",0.0)), float(origin.get("yaw",0.0))]
    ox, oy, oyaw = float(origin[0]), float(origin[1]), float(origin[2])
    negate = int(cfg.get("negate", 0))
    occ_th = float(cfg.get("occupied_thresh", 0.65))
    free_th = float(cfg.get("free_thresh", 0.196))

    # map_server 同等逻辑：negate=0 -> occ=(255-p)/255; negate=1 -> occ=p/255
    # occupied: occ > occupied_thresh
    occ_rows = []
    for row_img in range(h):  # image row: 0 top
        base = row_img * w
        row = []
        for x in range(w):
            p = pixels[base + x]
            occ = (p/255.0) if negate else ((255 - p)/255.0)
            row.append(occ > occ_th)
        occ_rows.append(row)

    # 翻转到“map坐标系”：y=0 在底部
    occ_rows = list(reversed(occ_rows))
    return res, (ox, oy, oyaw), occ_rows

def rect_decompose(occ_rows):
    h = len(occ_rows)
    w = len(occ_rows[0]) if h else 0
    rects = []
    active = {}  # key=(x0,width) -> (x0,width,y0,y1)
    for y in range(h):  # y from bottom
        row = occ_rows[y]
        runs = []
        x = 0
        while x < w:
            if row[x]:
                x0 = x
                while x < w and row[x]:
                    x += 1
                x1 = x - 1
                runs.append((x0, x1))
            else:
                x += 1

        new_active = {}
        for x0, x1 in runs:
            width = x1 - x0 + 1
            key = (x0, width)
            if key in active:
                rx0, rw, ry0, _ry1 = active[key]
                new_active[key] = (rx0, rw, ry0, y)
            else:
                new_active[key] = (x0, width, y, y)

        # finalize ones not continued
        for key, val in active.items():
            if key not in new_active:
                rects.append(val)
        active = new_active

    rects.extend(active.values())
    return rects  # list of (x0,width,y0,y1) in cell units

def write_model(out_dir: Path, model_name: str, rects, res: float, wall_h: float, with_visual: bool):
    out_dir.mkdir(parents=True, exist_ok=True)

    # model.config
    (out_dir / "model.config").write_text(
f"""<?xml version="1.0"?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>auto</name></author>
  <description>Auto-generated collision boxes from ROS occupancy map.</description>
</model>
""", encoding="utf-8")

    # model.sdf
    lines = []
    lines.append("<?xml version='1.0'?>")
    lines.append("<sdf version='1.6'>")
    lines.append(f"  <model name='{model_name}'>")
    lines.append("    <static>true</static>")
    lines.append("    <link name='link'>")

    for i,(x0,width,y0,y1) in enumerate(rects):
        height = (y1 - y0 + 1)
        sx = width * res
        sy = height * res
        sz = wall_h
        cx = (x0 + width/2.0) * res
        cy = (y0 + height/2.0) * res
        cz = wall_h / 2.0
        cname = f"occ_{i:04d}"
        # collision
        lines.append(f"      <collision name='{cname}'>")
        lines.append(f"        <pose>{cx:.6f} {cy:.6f} {cz:.6f} 0 0 0</pose>")
        lines.append("        <geometry>")
        lines.append("          <box>")
        lines.append(f"            <size>{sx:.6f} {sy:.6f} {sz:.6f}</size>")
        lines.append("          </box>")
        lines.append("        </geometry>")
        lines.append("      </collision>")

        if with_visual:
            vname = f"vis_{i:04d}"
            lines.append(f"      <visual name='{vname}'>")
            lines.append(f"        <pose>{cx:.6f} {cy:.6f} {cz:.6f} 0 0 0</pose>")
            lines.append("        <geometry>")
            lines.append("          <box>")
            lines.append(f"            <size>{sx:.6f} {sy:.6f} {sz:.6f}</size>")
            lines.append("          </box>")
            lines.append("        </geometry>")
            # 让它半透明，便于看到环境
            lines.append("        <material><ambient>0.8 0.3 0.1 0.15</ambient></material>")
            lines.append("        <transparency>0.85</transparency>")
            lines.append("      </visual>")

    lines.append("    </link>")
    lines.append("  </model>")
    lines.append("</sdf>")
    (out_dir / "model.sdf").write_text("\n".join(lines) + "\n", encoding="utf-8")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True, help="Path to map.yaml")
    ap.add_argument("--model-name", default="farm_map_collision")
    ap.add_argument("--out-model-dir", default="src/farm_gate_world/models")
    ap.add_argument("--wall-height", type=float, default=1.0)
    ap.add_argument("--with-visual", type=int, default=0, help="1: also generate visual boxes")
    args = ap.parse_args()

    map_yaml = Path(args.map).expanduser().resolve()
    cfg = load_map_yaml(map_yaml)
    img_rel = cfg.get("image", "map.pgm")
    img_path = (map_yaml.parent / img_rel).resolve()

    w, h, pixels = read_pgm(img_path)
    res, origin, occ_rows = build_occ_mask(cfg, w, h, pixels)
    rects = rect_decompose(occ_rows)

    out_root = Path(args.out_model_dir).expanduser()
    out_dir = out_root / args.model_name
    write_model(out_dir, args.model_name, rects, res, args.wall_height, bool(args.with_visual))

    # 统计
    occ_cells = sum(1 for row in occ_rows for v in row if v)
    covered = sum(width*(y1-y0+1) for (x0,width,y0,y1) in rects)
    print("=== Generated collision model ===")
    print("map:", map_yaml)
    print("image:", img_path)
    print("size (cells):", w, "x", h)
    print("resolution:", res)
    print("origin (x,y,yaw):", origin)
    print("occupied cells:", occ_cells, "covered:", covered, "rectangles:", len(rects))
    print("output model dir:", out_dir)
    print("")
    ox, oy, oyaw = origin
    print("Add this include to your .world (pose uses map origin):")
    print(f"  <include><uri>model://{args.model_name}</uri><pose>{ox} {oy} 0 0 0 {oyaw}</pose></include>")

if __name__ == "__main__":
    main()
