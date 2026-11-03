#!/usr/bin/env python3
"""Build the Nav2 keepout mask (keepout_mask.pgm + .yaml) from keepout_zones.yaml.

The mask has the SAME size, resolution and origin as the navigation map it names, so the KeepoutFilter can
look cells up 1:1. Cells inside a zone are black (occupied -> lethal cost), everything else white (free).
No dependencies beyond the standard library: the PGM is written by hand.
"""
import os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))


def load_yaml_simple(path):
    """Enough YAML for these two files: top-level 'key: value' scalars, a 'zones:' list of
    {name, polygon} items. (No PyYAML on every machine this runs on.)"""
    meta, zones, cur = {}, [], None
    for raw in open(path):
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        if line.startswith("  - name:"):
            cur = {"name": line.split(":", 1)[1].strip(), "polygon": []}; zones.append(cur)
        elif line.strip().startswith("polygon:") and cur is not None:
            nums = [float(v) for v in re.findall(r"-?\d+(?:\.\d+)?", line.split(":", 1)[1])]
            cur["polygon"] = list(zip(nums[0::2], nums[1::2]))
        elif not line.startswith(" ") and ":" in line:
            k, v = line.split(":", 1); meta[k.strip()] = v.strip()
    return meta, zones


def inside(x, y, poly):
    r = False; n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]; x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            r = not r
    return r


def main():
    zpath = os.path.join(HERE, "keepout_zones.yaml")
    meta, zones = load_yaml_simple(zpath)
    mpath = os.path.join(HERE, meta.get("map", "zone_one.yaml"))
    mmeta, _ = load_yaml_simple(mpath)
    res = float(mmeta["resolution"]); origin = [float(v) for v in re.findall(r"-?\d+(?:\.\d+)?", mmeta["origin"])]
    with open(os.path.join(HERE, mmeta["image"]), "rb") as f:        # size from the map's PGM header
        assert f.readline().strip() == b"P5", "map image must be binary PGM"
        tok = []
        while len(tok) < 3:
            line = f.readline()
            if not line.startswith(b"#"):
                tok += line.split()
        w, h = int(tok[0]), int(tok[1])
    free, occ = 254, 0
    img = bytearray([free]) * (w * h)
    for z in zones:
        xs = [p[0] for p in z["polygon"]]; ys = [p[1] for p in z["polygon"]]
        c0, c1 = max(0, int((min(xs) - origin[0]) / res)), min(w - 1, int((max(xs) - origin[0]) / res))
        r0, r1 = max(0, int((min(ys) - origin[1]) / res)), min(h - 1, int((max(ys) - origin[1]) / res))
        n = 0
        for r in range(r0, r1 + 1):                 # r counts up from the bottom row (map y); PGM rows go top-down
            y = origin[1] + (r + 0.5) * res
            for c in range(c0, c1 + 1):
                x = origin[0] + (c + 0.5) * res
                if inside(x, y, z["polygon"]):
                    img[(h - 1 - r) * w + c] = occ; n += 1
        print(f"{z['name']}: {n} cells ({n * res * res:.2f} m2)")
    out = os.path.join(HERE, "keepout_mask.pgm")
    with open(out, "wb") as f:
        f.write(f"P5\n# keepout mask for {mmeta['image']}: black = no-go\n{w} {h}\n255\n".encode()); f.write(bytes(img))
    with open(os.path.join(HERE, "keepout_mask.yaml"), "w") as f:
        f.write(f"image: keepout_mask.pgm\nmode: trinary\nresolution: {res}\norigin: [{origin[0]}, {origin[1]}, 0]\n"
                f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    print(f"wrote {out} ({w}x{h} @ {res} m, origin {origin[:2]})")


if __name__ == "__main__":
    main()
