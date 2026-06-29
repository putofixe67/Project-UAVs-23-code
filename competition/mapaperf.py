#!/usr/bin/env python3
"""2D map overview — obstacles, rover route, ArUco markers, landing pads."""
import os
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib import font_manager as _fm
from mapa import extrair_segmentos_stl as _parse_stl, PONTOS_ROVER

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
FIG_W, FIG_H, FIG_DPI = 19.2, 10.8, 200
C_WALL  = "#2C2C2C"
C_RPATH = "#BDBDBD"
C_BASE  = "#F4D03F"
C_BDER  = "#1D6A4A"
C_GRID  = "#E8E8E8"
C_TEXT  = "#2C2C2C"

sd  = os.path.dirname(os.path.abspath(__file__))
stl = os.path.join(sd, "icuas26_1.stl")

print("A ler geometria do mapa 3D (STL)...")
segs = _parse_stl(stl)

ARUCO = [
    (-5.9245,-1.3903),(7.6041,2.8088),(-3.8640,6.9430),(4.8932,-4.3532),
    (8.1615,8.7107),(4.8295,1.5269),(-2.6701,7.2078),(-7.6504,-8.5481),
    (1.2507,-6.5897),(0.5277,-4.3226),(7.6705,-5.2531),(-7.4942,-9.0051),
    (-7.7655,-9.0323),(-1.9260,-2.1796),(-1.7333,-4.9403),(-2.0129,-2.0421),
    (0.4546,-3.8881),(6.9151,-8.2716),(-5.8848,-1.2585),(5.9808,5.9191),
    (6.5083,-2.1766),(-1.2361,-4.8519),(-2.2373,1.9367),(6.0451,6.4071),
    (-2.1589,7.1959),(-8.6736,4.2784),(0.7506,-3.8684),(-8.8289,4.4761),
]
LANDING = [
    (-6.068,-1.390,"Box 2"),(7.748,2.808,"Box 3"),(-3.721,6.942,"Box 4"),
    (4.750,-4.353,"Box 5"),(8.018,8.711,"Box 6"),(4.686,1.528,"Box 7"),
    (-2.814,7.208,"Box 8"),(-7.651,-8.405,"Box 9"),(1.250,-6.733,"Box 10"),
]

plt.rcParams.update({
    "font.family":     FONT_MAIN,
    "axes.linewidth":  0.8,
    "xtick.direction": "in",
    "ytick.direction": "in",
    "xtick.major.pad": 6,
    "ytick.major.pad": 6,
})

fig = plt.figure(figsize=(FIG_W, FIG_H), facecolor="white", dpi=FIG_DPI)

fig.text(0.50, 0.976, "UAV Relay Network: Operation Map — ICUAS 2026",
         ha="center", va="top", fontsize=26, fontweight="bold",
         color=C_TEXT, fontfamily=FONT_MAIN)
fig.text(0.50, 0.926,
         f"24 Obstacle Pillars  ·  Rover Route  ·  "
         f"{len(ARUCO)} ArUco Markers  ·  {len(LANDING)} Landing Platforms",
         ha="center", va="top", fontsize=15, color="#606060",
         fontstyle="italic", fontfamily=FONT_MAIN)

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

# Arena boundary
ax.add_patch(plt.Rectangle((-10,-10), 20, 20, edgecolor="#CCCCCC",
             linestyle=":", facecolor="none", linewidth=1.0, zorder=1))

# Charging area
ax.add_patch(plt.Rectangle((-1.5,-1.5), 3.0, 3.0, linewidth=1.2,
             edgecolor="#1D6A4A", facecolor="#D5F5E3", alpha=0.6, zorder=2))

# Obstacle walls
for seg in segs:
    ax.plot([seg[0][0], seg[1][0]], [seg[0][1], seg[1][1]],
            color=C_WALL, linewidth=1.5, zorder=3)

# Rover route
rover_x = [p[0] for p in PONTOS_ROVER]
rover_y = [p[1] for p in PONTOS_ROVER]
ax.plot(rover_x, rover_y, color=C_RPATH, lw=1.4, ls="--", alpha=0.9, zorder=4)
ax.scatter(rover_x[0], rover_y[0], color="#C0392B", s=120, marker="X",
           linewidths=1.2, zorder=7, label="Rover start")

# Ground station
ax.scatter(0, 0, s=340, marker="*", c=C_BASE,
           edgecolors=C_BDER, linewidths=1.3, zorder=8)

# ArUco markers
ax.scatter([p[0] for p in ARUCO], [p[1] for p in ARUCO],
           color="royalblue", s=35, marker="o", zorder=6)

# Landing platforms
ax.scatter([s[0] for s in LANDING], [s[1] for s in LANDING],
           color="#C0392B", s=70, marker="s",
           edgecolors="#7B241C", linewidths=1.2, zorder=6)
for x, y, name in LANDING:
    ax.text(x + 0.22, y + 0.22, name, color="#922B21",
            fontsize=10, fontfamily=FONT_MAIN)

# ── Right panel ───────────────────────────────────────────────────────────────
leg_handles = [
    plt.Line2D([0],[0], color=C_WALL, lw=1.5, label="Obstacle walls"),
    plt.Line2D([0],[0], color=C_RPATH, lw=1.6, ls="--", label="Rover route"),
    plt.Line2D([0],[0], marker="X", color="#C0392B", ms=11, ls="none",
               markeredgewidth=1.2, label="Rover start"),
    plt.Line2D([0],[0], marker="*", color=C_BASE, ms=16,
               markeredgecolor=C_BDER, markeredgewidth=1.0, ls="none",
               label="Ground station"),
    mpatches.Patch(facecolor="#D5F5E3", edgecolor="#1D6A4A",
                   linewidth=1.2, label="Charging area"),
    plt.Line2D([0],[0], marker="o", color="royalblue", ms=8,
               ls="none", label="ArUco markers"),
    plt.Line2D([0],[0], marker="s", color="#C0392B", ms=9,
               markeredgecolor="#7B241C", markeredgewidth=1.0,
               ls="none", label="Landing platforms"),
]

right_ax = fig.add_axes([0.680, 0.35, 0.115, 0.53])
right_ax.set_facecolor("white"); right_ax.set_xlim(0,1); right_ax.set_ylim(0,1)
right_ax.set_xticks([]); right_ax.set_yticks([])
for sp in right_ax.spines.values():
    sp.set_edgecolor("#CCCCCC"); sp.set_linewidth(0.8)

right_ax.text(0.06, 0.97, "Legend", va="top", ha="left", fontsize=13,
              fontweight="bold", fontfamily=FONT_MAIN, color=C_TEXT)
right_ax.legend(handles=leg_handles, bbox_to_anchor=(0.06, 0.91),
                loc="upper left", bbox_transform=right_ax.transAxes,
                fontsize=13, frameon=False, handlelength=1.8,
                handletextpad=0.5, labelspacing=0.45, borderpad=0.0, ncol=1)

right_ax.axhline(y=0.37, color="#DDDDDD", lw=0.9, xmin=0.05, xmax=0.95)
right_ax.text(0.06, 0.35, "Map Info", va="top", ha="left", fontsize=13,
              fontweight="bold", fontfamily=FONT_MAIN, color=C_TEXT)
right_ax.text(0.06, 0.28, (
    f"Pillars :  24\n"
    f"Waypt.  :  {len(PONTOS_ROVER)}\n"
    f"ArUco   :  {len(ARUCO)}\n"
    f"Landing :  {len(LANDING)}\n"
    f"Arena   :  20×20 m"
), va="top", ha="left", fontsize=12, fontfamily=FONT_MONO,
   color=C_TEXT, linespacing=1.8)

fig.text(0.50, 0.010,
         "Urban obstacle field  ·  Rover speed 0.5 m/s  ·  "
         "Base station at origin  ·  Arena 20 × 20 m",
         ha="center", va="bottom", fontsize=12, color="#777777",
         fontstyle="italic", fontfamily=FONT_MAIN)

out = os.path.join(sd, "mapa_completo_relatorio.png")
plt.savefig(out, dpi=FIG_DPI, facecolor="white", bbox_inches="tight")
print(f"Guardado em '{out}'.")
plt.show()
