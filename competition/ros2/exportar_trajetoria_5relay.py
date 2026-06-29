#!/usr/bin/env python3
"""Exporta trajetoria feedforward (modo 5 relés livres) para trajetoria_relay_5.npz."""
import os, sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from mapa import construir_mapa, PONTOS_ROVER, SIM_FPS, ROVER_SPEED, V_MAX, N_RELES
from planeador import planear, suavizar

LEAD     = 300
FOLGA_MIN = 0.30
ALT_VOO  = 1.0


def main():
    sd  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
    d   = construir_mapa(os.path.join(sd, "icuas26_1.stl"))
    nf_lap = d.get("nf_lap", len(d["rover_path"]))
    FD, FR, des = planear(d["coer"], d["SN"], d["rover_path"],
                          d["CEN"], d["RAD"], LEAD, FOLGA_MIN)
    assert len(des) == 0, f"trajetoria com {len(des)} desligamentos — não exportar"
    FD = suavizar(FD)
    rota = np.array(PONTOS_ROVER, float)
    acc  = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(rota, axis=0), axis=1))])
    out  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trajetoria_relay_5.npz")
    np.savez(out,
             reles_xy=np.array([[FD[t][i] for i in range(N_RELES)] for t in range(nf_lap)]),
             rota_xy=rota, acc=acc,
             sim_fps=SIM_FPS, rover_speed=ROVER_SPEED, v_max=V_MAX,
             n_reles=N_RELES, alt_voo=ALT_VOO)
    print(f"exportado {out}: T={nf_lap} frames, {N_RELES} relés livres, 0 desligamentos")


if __name__ == "__main__":
    main()
