#!/usr/bin/env python3
import os
import numpy as np
from mapa import construir_mapa, corredor_lazy
from planeador import planear_sombra, metricas, suavizar
from render import render_mp4

LEAD            = 500
N_RELES_LIVRES  = 4
FOLGA_MIN       = 0.40
SHADOW_HOP_COST = 3.0


def validar(FD, des, CEN, RAD, nf):
    folga = sep = 1e9
    for t in range(nf):
        for p in FD[t][:N_RELES_LIVRES]:
            folga = min(folga, float((np.linalg.norm(CEN - p, axis=1) - RAD).min()))
        for i in range(len(FD[t])):
            for j in range(i + 1, len(FD[t])):
                sep = min(sep, float(np.linalg.norm(FD[t][i] - FD[t][j])))
    nd, vmax = metricas(FD, des, nf)
    print(f"  disconnections = {nd}\n  v_max = {vmax:.4f} m/s"
          f"\n  clearance = {folga:.3f} m\n  min sep = {sep:.3f} m")
    return vmax <= 1.5 + 1e-6 and folga >= 0


def main():
    sd = os.path.dirname(os.path.abspath(__file__))
    d  = construir_mapa(os.path.join(sd, "icuas26_1.stl"))
    nf = len(d["rover_path"])
    print(f"Map: {len(d['CEN'])} pillars | {len(d['SN'])} nodes | {nf} frames")
    print(f"Building shadow corridor (hop_cost={SHADOW_HOP_COST})...")
    coer_shadow = corredor_lazy(d["SN"], d["adj"], d["rover_path"], d["CEN"], d["RAD"],
                                hop_cost=SHADOW_HOP_COST)
    print(f"Planning shadow (LEAD={LEAD}, {N_RELES_LIVRES} relays):")
    FD, FR, des = planear_sombra(coer_shadow, d["SN"], d["rover_path"], d["CEN"], d["RAD"],
                                 LEAD, N_RELES_LIVRES, FOLGA_MIN)
    err = max(np.linalg.norm(FD[t][N_RELES_LIVRES] - d["rover_path"][t]) for t in range(nf))
    print(f"  shadow↔rover max error = {err:.2e} m")
    if not validar(FD, des, d["CEN"], d["RAD"], nf):
        print("VALIDATION FAILED — aborting."); return
    out = os.path.join(sd, "drone_relay_sombra.mp4")
    n, stride = render_mp4(d["segs"], FD, FR, d["rover_path"], out, shadow_mode=True)
    print(f"Saved: {n} frames (stride={stride}) @ 60 fps  →  {out}")


if __name__ == "__main__":
    main()
