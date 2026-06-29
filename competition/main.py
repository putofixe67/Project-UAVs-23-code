#!/usr/bin/env python3
import os
import numpy as np
from mapa import construir_mapa, N_RELES
from planeador import planear, metricas
from render import render_mp4

LEAD = 300


def validate(FD, des, CEN, RAD, nf):
    clearance = sep = 1e9
    for t in range(nf):
        pos = FD[t]
        for p in pos:
            clearance = min(clearance, float((np.linalg.norm(CEN - p, axis=1) - RAD).min()))
        for i in range(N_RELES):
            for j in range(i + 1, N_RELES):
                sep = min(sep, float(np.linalg.norm(pos[i] - pos[j])))
    nd, vmax = metricas(FD, des, nf)
    print(f"  disconnections      = {nd}")
    print(f"  v_max               = {vmax:.4f} m/s   (limit 1.5 m/s)")
    print(f"  obstacle clearance  = {clearance:.3f} m    (>= 0 m required)")
    print(f"  min UAV separation  = {sep:.3f} m    (>= 0.5 m required)")
    return nd == 0 and vmax <= 1.5 + 1e-6 and clearance >= 0 and sep >= 0.5 - 1e-2


def main():
    sd = os.path.dirname(os.path.abspath(__file__))
    d  = construir_mapa(os.path.join(sd, "icuas26_1.stl"))
    nf = len(d["rover_path"])
    print(f"Map: {len(d['CEN'])} pillars | {len(d['SN'])} nodes | {nf} frames @ 15 fps")
    print(f"Planning (LEAD={LEAD}):")
    FD, FR, des = planear(d["coer"], d["SN"], d["rover_path"], d["CEN"], d["RAD"], LEAD)
    if not validate(FD, des, d["CEN"], d["RAD"], nf):
        print("VALIDATION FAILED — aborting."); return
    out = os.path.join(sd, "drone_relay.mp4")
    n, stride = render_mp4(d["segs"], FD, FR, d["rover_path"], out)
    print(f"Saved: {n} frames (stride={stride}) @ 60 fps  →  {out}")


if __name__ == "__main__":
    main()
