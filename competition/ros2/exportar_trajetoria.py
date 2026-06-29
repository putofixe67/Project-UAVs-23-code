#!/usr/bin/env python3
"""Exporta trajetoria feedforward (modo sombra) para trajetoria_relay.npz."""
import os, sys
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from mapa import construir_mapa, PONTOS_ROVER, SIM_FPS, ROVER_SPEED, V_MAX
from planeador import planear_sombra, suavizar

LEAD, N_RELES_LIVRES, FOLGA_MIN = 300, 4, 0.40
ALT_VOO, ALT_SOMBRA = 1.0, 0.8


def main():
    sd  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
    d   = construir_mapa(os.path.join(sd, "icuas26_1.stl"))
    nf_lap = d.get("nf_lap", len(d["rover_path"]))
    FD, FR, des = planear_sombra(d["coer"], d["SN"], d["rover_path"],
                                 d["CEN"], d["RAD"], LEAD, N_RELES_LIVRES, FOLGA_MIN)
    assert len(des) == 0, f"trajetoria com {len(des)} desligamentos — não exportar"
    FD = suavizar(FD)  # mesma suavização da animação — setpoints seguros para o Gazebo
    rota = np.array(PONTOS_ROVER, float)
    acc  = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(rota, axis=0), axis=1))])
    out  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trajetoria_relay.npz")
    np.savez(out,
             reles_xy=np.array([[FD[t][i] for i in range(N_RELES_LIVRES)] for t in range(nf_lap)]),
             rover_xy=np.array([d["rover_path"][t] for t in range(nf_lap)]),
             rota_xy=rota, acc=acc,
             sim_fps=SIM_FPS, rover_speed=ROVER_SPEED, v_max=V_MAX,
             n_reles=N_RELES_LIVRES, alt_voo=ALT_VOO, alt_sombra=ALT_SOMBRA)
    print(f"exportado {out}: T={nf_lap} frames, {N_RELES_LIVRES} relés + 1 sombra, 0 desligamentos")


if __name__ == "__main__":
    main()
