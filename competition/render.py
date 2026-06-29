#!/usr/bin/env python3
"""Shared renderer for both relay modes (standard and shadow).
Call render_mp4(..., shadow_mode=True) for the shadow-drone variant."""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import PathPatch
from matplotlib.path import Path
from matplotlib.transforms import Affine2D, Bbox
from matplotlib.collections import LineCollection
from matplotlib.animation import FuncAnimation
from matplotlib import font_manager as _fm

from mapa import BASE, PONTOS_ROVER, DT

for _f in (
    "/usr/share/fonts/urw-base35/NimbusSans-Regular.otf",
    "/usr/share/fonts/urw-base35/NimbusSans-Bold.otf",
    "/usr/share/fonts/urw-base35/NimbusSans-Italic.otf",
    "/usr/share/fonts/urw-base35/NimbusSans-BoldItalic.otf",
):
    if os.path.exists(_f):
        _fm.fontManager.addfont(_f)

FONT_MAIN = "Nimbus Sans"
FONT_MONO = "Liberation Mono"
OUT_FPS      = 60
DURACAO_ALVO = 40
FIG_W, FIG_H = 19.2, 10.8
FIG_DPI      = 200

C_WALL   = "#2C2C2C"
C_RPATH  = "#BDBDBD"
C_LINK   = "#1A5276"
C_LNOFF  = "#922B21"
C_BASE   = "#F4D03F"
C_BDER   = "#1D6A4A"
C_ROVER  = "#CA6F1E"
C_RDER   = "#7D3C00"
C_DRONE  = "#00897B"
C_DDER   = "#004D40"
C_SHADOW = "#7B2D8B"
C_SHDER  = "#4A1A55"
C_GRID   = "#E8E8E8"
C_TEXT   = "#2C2C2C"


def _circle(cx, cy, r, n, verts, codes):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    pts = np.column_stack((cx + r * np.cos(a), cy + r * np.sin(a)))
    verts.append(pts[0].tolist()); codes.append(Path.MOVETO)
    for p in pts[1:]: verts.append(p.tolist()); codes.append(Path.LINETO)
    verts.append(pts[0].tolist()); codes.append(Path.CLOSEPOLY)


def _quadcopter_marker(arm=0.42, r_rot=0.17, r_body=0.09):
    verts, codes = [], []
    _circle(0.0, 0.0, r_body, 16, verts, codes)
    for deg in (45, 135, 225, 315):
        rad = np.deg2rad(deg)
        cx, cy = arm * np.cos(rad), arm * np.sin(rad)
        verts.append([r_body * np.cos(rad), r_body * np.sin(rad)]); codes.append(Path.MOVETO)
        verts.append([cx, cy]); codes.append(Path.LINETO)
        _circle(cx, cy, r_rot, 16, verts, codes)
    return Path(np.array(verts, float), codes)


def _rover_marker(w=0.80, h=0.44, wr=0.13):
    verts, codes = [], []
    corners = [(-w/2, -h/2), (w/2, -h/2), (w/2, h/2), (-w/2, h/2)]
    verts.append(list(corners[0])); codes.append(Path.MOVETO)
    for c in corners[1:]: verts.append(list(c)); codes.append(Path.LINETO)
    verts.append(list(corners[0])); codes.append(Path.CLOSEPOLY)
    for wx in (-w/2 + wr, w/2 - wr):
        for wy in (-h/2, h/2):
            _circle(wx, wy, wr, 12, verts, codes)
    return Path(np.array(verts, float), codes)


QUAD_MK  = _quadcopter_marker()
ROVER_MK = _rover_marker()


def _compute_crop(fig):
    fig.canvas.draw()
    tb  = fig.get_tightbbox(fig.canvas.get_renderer())
    fx0 = max(0.0, tb.x0 / FIG_W); fy0 = max(0.0, tb.y0 / FIG_H)
    fx1 = min(1.0, tb.x1 / FIG_W); fy1 = min(1.0, tb.y1 / FIG_H)
    pf  = 30 / (FIG_W * FIG_DPI);  pfh = 30 / (FIG_H * FIG_DPI)
    fx0 = max(0.0, fx0 - pf);  fy0 = max(0.0, fy0 - pfh)
    fx1 = min(1.0, fx1 + pf);  fy1 = min(1.0, fy1 + pfh)
    W, H = int(FIG_W * FIG_DPI), int(FIG_H * FIG_DPI)
    cx   = int(fx0 * W);             cy  = int((1.0 - fy1) * H)
    cw   = int((fx1 - fx0) * W) & ~1; ch = int((fy1 - fy0) * H) & ~1
    bbox = Bbox([[fx0 * FIG_W, fy0 * FIG_H], [fx1 * FIG_W, fy1 * FIG_H]])
    return bbox, f"crop={cw}:{ch}:{cx}:{cy}", (cw, ch, cx, W - cx - cw)


def render_mp4(segs, FD, FR, rover_path, output_path,
               shadow_mode=False, screenshot_only=False):
    n_reles = len(FD[0]) - 1 if shadow_mode else len(FD[0])
    nf      = len(rover_path)
    stride  = max(1, round(nf / (OUT_FPS * DURACAO_ALVO)))
    idx     = list(range(0, nf, stride))

    def _heading(t):
        for dt in range(1, 9):
            if t + dt < nf:
                dx = rover_path[t+dt][0] - rover_path[t][0]
                dy = rover_path[t+dt][1] - rover_path[t][1]
                if dx*dx + dy*dy > 1e-8: return np.degrees(np.arctan2(dy, dx))
        for dt in range(1, 9):
            if t - dt >= 0:
                dx = rover_path[t][0] - rover_path[t-dt][0]
                dy = rover_path[t][1] - rover_path[t-dt][1]
                if dx*dx + dy*dy > 1e-8: return np.degrees(np.arctan2(dy, dx))
        return 0.0

    rover_headings = [_heading(t) for t in range(nf)]

    plt.rcParams.update({
        "font.family": FONT_MAIN, "axes.linewidth": 0.8,
        "xtick.direction": "in", "ytick.direction": "in",
        "xtick.major.pad": 6,    "ytick.major.pad": 6,
    })
    fig = plt.figure(figsize=(FIG_W, FIG_H), facecolor="white", dpi=FIG_DPI)

    title = ("UAV Relay Network: Shadow-Drone Coverage Mode" if shadow_mode
             else "UAV Relay Network: Feed-Forward Coverage Planning")
    subtitle = ("ICUAS 2026  ·  4 Relay UAVs  +  1 Shadow Drone  ·  1 Ground Rover  ·  Urban Obstacle Field"
                if shadow_mode else
                "ICUAS 2026  ·  5 Quadrotors  ·  1 Ground Rover  ·  Urban Obstacle Field")
    fig.text(0.50, 0.976, title, ha="center", va="top",
             fontsize=26, fontweight="bold", color=C_TEXT, fontfamily=FONT_MAIN)
    fig.text(0.50, 0.926, subtitle, ha="center", va="top",
             fontsize=15, color="#606060", fontstyle="italic", fontfamily=FONT_MAIN)

    ax = fig.add_axes([0.04, 0.09, 0.82, 0.81])
    ax.set_xlim(-11, 11); ax.set_ylim(-11, 11)
    ax.set_aspect("equal"); ax.set_facecolor("white")
    ax.grid(True, color=C_GRID, linewidth=0.6, linestyle="--", zorder=0)
    ax.tick_params(labelsize=14, colors="#555555", length=4)
    ax.set_xlabel("x  (m)", fontsize=15, color=C_TEXT, labelpad=4)
    ax.set_ylabel("y  (m)", fontsize=15, color=C_TEXT, labelpad=6)
    ax.spines["left"].set_edgecolor("#BBBBBB");   ax.spines["left"].set_linewidth(0.8)
    ax.spines["bottom"].set_edgecolor("#BBBBBB"); ax.spines["bottom"].set_linewidth(0.8)
    ax.spines["right"].set_visible(False);         ax.spines["top"].set_visible(False)

    ax.add_collection(LineCollection(
        [[(s[0][0], s[0][1]), (s[1][0], s[1][1])] for s in segs],
        colors=C_WALL, linewidths=1.5, zorder=2))
    ax.plot([p[0] for p in PONTOS_ROVER], [p[1] for p in PONTOS_ROVER],
            color=C_RPATH, lw=1.1, ls="--", alpha=0.65, zorder=1)
    ax.scatter(*BASE, s=340, marker="*", c=C_BASE,
               edgecolors=C_BDER, linewidths=1.3, zorder=6)

    link_line, = ax.plot([], [], color=C_LINK, lw=1.0, solid_capstyle="round", zorder=3)
    drone_mk,  = ax.plot([], [], marker=QUAD_MK, ms=8, color=C_DRONE,
                         markeredgecolor=C_DDER, markeredgewidth=0.6, ls="none", zorder=5)
    shadow_mk  = (ax.plot([], [], marker=QUAD_MK, ms=9, color=C_SHADOW,
                           markeredgecolor=C_SHDER, markeredgewidth=0.7, ls="none", zorder=6)[0]
                  if shadow_mode else None)
    rover_patch = PathPatch(ROVER_MK, fc=C_ROVER, ec=C_RDER, lw=0.9, zorder=5)
    ax.add_patch(rover_patch)

    _box = dict(boxstyle="round,pad=0.45", facecolor="white", edgecolor="#CCCCCC",
                linewidth=0.9, alpha=0.93)
    time_txt  = ax.text(0.985, 0.974, "t =   0.0 s", transform=ax.transAxes,
                        ha="right", va="top", fontsize=15, color=C_TEXT,
                        fontfamily=FONT_MAIN, bbox=_box)
    conn_txt  = ax.text(0.016, 0.974, "●  LINK ACTIVE", transform=ax.transAxes,
                        ha="left", va="top", fontsize=15, fontweight="bold", color=C_LINK,
                        fontfamily=FONT_MAIN,
                        bbox=dict(boxstyle="round,pad=0.45", facecolor="white",
                                  edgecolor=C_LINK, linewidth=0.9, alpha=0.93))
    tgt_label = "relays → shadow" if shadow_mode else "UAVs → rover"
    drones_txt = ax.text(0.016, 0.916, f"0 / {n_reles}  {tgt_label}",
                         transform=ax.transAxes, ha="left", va="top",
                         fontsize=13, color="#444444", fontfamily=FONT_MAIN,
                         bbox=dict(boxstyle="round,pad=0.35", facecolor="white",
                                   edgecolor="#CCCCCC", linewidth=0.8, alpha=0.93))

    drone_label = "Relay UAV" if shadow_mode else "Quadrotor (UAV)"
    leg_handles = [
        plt.Line2D([0], [0], marker="o", markerfacecolor="white", ms=13,
                   markeredgecolor="#555555", markeredgewidth=1.5, ls="none", label="Obstacle pillar"),
        plt.Line2D([0], [0], color=C_RPATH, lw=1.6, ls="--", label="Rover waypoints"),
        plt.Line2D([0], [0], marker="*", color=C_BASE, ms=16, markeredgecolor=C_BDER,
                   markeredgewidth=1.0, ls="none", label="Base station"),
        plt.Line2D([0], [0], marker=QUAD_MK, color=C_DRONE, ms=13, markeredgecolor=C_DDER,
                   markeredgewidth=0.8, ls="none", label=drone_label),
    ]
    if shadow_mode:
        leg_handles.append(
            plt.Line2D([0], [0], marker=QUAD_MK, color=C_SHADOW, ms=13,
                       markeredgecolor=C_SHDER, markeredgewidth=0.8, ls="none", label="Shadow drone"))
    leg_handles += [
        plt.Line2D([0], [0], marker=ROVER_MK, color=C_ROVER, ms=14,
                   markeredgecolor=C_RDER, markeredgewidth=0.9, ls="none", label="Ground rover"),
        plt.Line2D([0], [0], color=C_LINK, lw=1.6, label="Relay link"),
    ]

    # shadow mode has one extra legend entry so the panel is taller
    panel_y = 0.40 if shadow_mode else 0.44
    panel_h = 0.45 if shadow_mode else 0.41
    right_ax = fig.add_axes([0.680, panel_y, 0.115, panel_h])
    right_ax.set_facecolor("white"); right_ax.set_xlim(0, 1); right_ax.set_ylim(0, 1)
    right_ax.set_xticks([]); right_ax.set_yticks([])
    for sp in right_ax.spines.values():
        sp.set_edgecolor("#CCCCCC"); sp.set_linewidth(0.8)
    right_ax.text(0.06, 0.97, "Legend", va="top", ha="left", fontsize=13,
                  fontweight="bold", fontfamily=FONT_MAIN, color=C_TEXT)
    right_ax.legend(handles=leg_handles, bbox_to_anchor=(0.06, 0.91), loc="upper left",
                    bbox_transform=right_ax.transAxes, fontsize=13, frameon=False,
                    handlelength=1.8, handletextpad=0.5, labelspacing=0.45,
                    borderpad=0.0, ncol=1)
    hline_y  = 0.46  if shadow_mode else 0.475
    info_top = 0.44  if shadow_mode else 0.455
    info_y   = 0.37  if shadow_mode else 0.385
    right_ax.axhline(y=hline_y, color="#DDDDDD", lw=0.9, xmin=0.05, xmax=0.95)
    right_ax.text(0.06, info_top, "Simulation Info", va="top", ha="left", fontsize=13,
                  fontweight="bold", fontfamily=FONT_MAIN, color=C_TEXT)
    info_txt = right_ax.text(0.06, info_y, "", va="top", ha="left", fontsize=12,
                              fontfamily=FONT_MONO, color=C_TEXT, linespacing=1.8)

    footer = ("Algorithm: lazy Dijkstra with sticky re-planning  ·  v_max = 1.5 m/s  "
              + ("·  shadow drone directly above rover  " if shadow_mode else "")
              + "·  obstacle clearance ≥ 0.40 m")
    fig.text(0.50, 0.010, footer, ha="center", va="bottom",
             fontsize=12, color="#777777", fontstyle="italic", fontfamily=FONT_MAIN)

    def upd(k):
        t = idx[k]
        nos, arestas, ligado = FR[t]
        xs, ys = [], []
        for u, v in arestas:
            xs += [nos[u][0], nos[v][0], np.nan]
            ys += [nos[u][1], nos[v][1], np.nan]
        link_line.set_data(xs, ys)

        if shadow_mode:
            drone_mk.set_data([p[0] for p in FD[t][:n_reles]],
                              [p[1] for p in FD[t][:n_reles]])
            sh = FD[t][n_reles]
            shadow_mk.set_data([sh[0]], [sh[1]])
            shadow_idx = len(nos) - 2; rover_idx = len(nos) - 1
            count = sum(1 for u, v in arestas
                        if shadow_idx in (u, v) and rover_idx not in (u, v) and 0 not in (u, v))
        else:
            drone_mk.set_data([p[0] for p in FD[t]], [p[1] for p in FD[t]])
            rover_idx = len(nos) - 1
            count = sum(1 for u, v in arestas if rover_idx in (u, v))

        rover_patch.set_transform(
            Affine2D().scale(0.60).rotate_deg(rover_headings[t])
            .translate(rover_path[t][0], rover_path[t][1]) + ax.transData)

        sim_t = t * DT
        time_txt.set_text(f"t = {sim_t:5.1f} s")
        if ligado:
            conn_txt.set_text("●  LINK ACTIVE"); conn_txt.set_color(C_LINK)
            conn_txt.get_bbox_patch().set_edgecolor(C_LINK); relay_status = "active"
        else:
            conn_txt.set_text("✖  LINK BROKEN"); conn_txt.set_color(C_LNOFF)
            conn_txt.get_bbox_patch().set_edgecolor(C_LNOFF); relay_status = "BROKEN"
        drones_txt.set_text(f"{count} / {n_reles}  {tgt_label}")
        info_txt.set_text(
            f"Frame :  {k + 1:>4} / {len(idx)}\n"
            f"Time  :  {sim_t:>5.1f} s\n"
            f"Relay :  {relay_status}\n"
            f"{'UAV>S' if shadow_mode else 'UAV>R'} :  {count} / {n_reles}\n"
            f"FPS   :  {OUT_FPS}"
        )

    _t22 = round(22.0 / DT)
    _k22 = min(range(len(idx)), key=lambda k: abs(idx[k] - _t22))
    upd(_k22)

    png = output_path.replace(".mp4", "_t22s.png")
    bbox_in, vf_str, (cw, ch, cx, right_trim) = _compute_crop(fig)
    fig.savefig(png, dpi=FIG_DPI, facecolor="white", bbox_inches=bbox_in)
    print(f"Screenshot: t={idx[_k22] * DT:.1f} s  →  {png}")
    print(f"Crop: {cw}×{ch} px  (trimmed {cx}px left, {right_trim}px right)")

    if screenshot_only:
        plt.close(fig)
        return len(idx), stride

    ani = FuncAnimation(fig, upd, frames=len(idx), interval=1000 / OUT_FPS, blit=False)
    ani.save(output_path, writer="ffmpeg", fps=OUT_FPS, dpi=FIG_DPI,
             extra_args=["-vcodec", "mpeg4", "-q:v", "3", "-pix_fmt", "yuv420p", "-vf", vf_str])
    plt.close(fig)
    return len(idx), stride
