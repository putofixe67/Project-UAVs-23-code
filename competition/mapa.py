#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Mapa e corredor da rede de relay (ICUAS26).
Constroi pilares -> nos -> grafo de navegacao -> corredor (Dijkstra sticky lazy).
Guarda tudo em cache (pickle) para iterar depressa no planeador."""
import os, heapq, struct, pickle, numpy as np

# ----- constantes partilhadas -----
# Folga de navegacao ABSOLUTA (nao proporcional ao raio do pilar): um Crazyflie
# tem tamanho fixo, logo precisa sempre da mesma distancia minima a superficie do
# pilar, independentemente de quao grande este e. = raio do drone + buffer.
MARGEM_NAV  = 0.40             # m, da SUPERFICIE do pilar (keep-out = RAD + MARGEM_NAV)
BASE        = np.array([0.0, 0.0])
N_RELES     = 5                # 5 reles livres; rover sem radio
ROVER_SPEED = 0.5
V_MAX       = 1.5
D_MIN       = 0.50             # esfera de 0.5 m de diametro por drone (centros >= 0.5 m)
PEN         = 1.4
SIM_FPS     = 15
DT          = 1.0 / SIM_FPS
STEP        = V_MAX * DT        # 0.1 m/frame

PONTOS_ROVER = [
    [-1.1248,-0.4892],[-0.1037,8.4403],[-7.6427,6.9194],[-4.5141,3.1173],
    [-8.2293,-1.8580],[-2.9715,-2.6835],[-4.4918,-3.4416],[-5.4918,-7.4416],
    [1.7864,-8.3107],[4.7370,-7.5750],[2.5034,-1.8580],[7.2615,0.9230],
    [5.7841,3.0087],[7.9241,7.2236],[3.3887,8.0818],[1.4605,-1.9448],[-1.1248,-0.4892]]

# ----- geometria vectorizada -----
def _sd(A, B, CEN):
    A = np.asarray(A, float); B = np.asarray(B, float); d = B - A; L2 = d.dot(d)
    if L2 < 1e-12: return np.linalg.norm(CEN - A, axis=1)
    t = np.clip((CEN - A) @ d / L2, 0, 1)
    return np.linalg.norm(CEN - (A + t[:, None] * d), axis=1)

def los_radio(A, B, CEN, RAD):              # oclusao pura (sem margem)
    return bool(np.all(_sd(A, B, CEN) >= RAD - 1e-9))

def corredor_nav(A, B, CEN, RAD):           # navegacao com margem absoluta
    lim = RAD + MARGEM_NAV
    return bool(np.all(_sd(A, B, CEN) >= lim - 1e-9))

# ----- mapa: STL -> pilares -> nos -> grafo -----
def extrair_segmentos_stl(fn, z=1.0):
    s = []
    with open(fn, "rb") as f:
        f.read(80); n = struct.unpack("<I", f.read(4))[0]
        for _ in range(n):
            dd = f.read(50)
            if len(dd) < 50: break
            u = struct.unpack("<12fH", dd); v1, v2, v3 = u[3:6], u[6:9], u[9:12]; pts = []
            for (e1, e2) in [(v1, v2), (v2, v3), (v3, v1)]:
                if (e1[2] <= z <= e2[2]) or (e2[2] <= z <= e1[2]):
                    if e1[2] != e2[2]:
                        t = (z - e1[2]) / (e2[2] - e1[2])
                        pts.append((e1[0]+t*(e2[0]-e1[0]), e1[1]+t*(e2[1]-e1[1])))
            if len(pts) >= 2: s.append((pts[0], pts[1]))
    return s

def extrair_pilares(segs, thr=0.6):
    from scipy.cluster.hierarchy import fcluster, linkage
    pts = []
    for p1, p2 in segs: pts.append(p1); pts.append(p2)
    pts = np.array(pts); uniq = []
    for p in pts:
        if not uniq or np.min(np.linalg.norm(np.array(uniq) - p, axis=1)) > 0.03: uniq.append(p)
    uniq = np.array(uniq); Z = linkage(uniq, method="single")
    lab = fcluster(Z, t=thr, criterion="distance"); cen, rad = [], []
    for k in range(1, lab.max() + 1):
        cl = uniq[lab == k]; ce = cl.mean(0)
        cen.append(ce); rad.append(float(np.max(np.linalg.norm(cl - ce, axis=1))))
    return np.array(cen), np.array(rad)

def gerar_nos(CEN, RAD, por=32):
    nos = []; ang = np.linspace(0, 2*np.pi, por, endpoint=False)
    for ce, r in zip(CEN, RAD):
        lim = r + MARGEM_NAV; R = lim * 1.05
        for a in ang:
            p = ce + R * np.array([np.cos(a), np.sin(a)])
            if np.all(np.linalg.norm(CEN - p, axis=1) >= lim - 1e-6): nos.append(p)
    return nos

def construir_matriz(SN, CEN, RAD):
    N = len(SN); adj = [[] for _ in range(N)]
    for i in range(N):
        for j in range(i+1, N):
            if corredor_nav(SN[i], SN[j], CEN, RAD):
                w = float(np.linalg.norm(SN[i] - SN[j])); adj[i].append((j, w)); adj[j].append((i, w))
    return adj

# ----- corredor: Dijkstra sticky + replaneamento preguicoso -----
def dijkstra_sticky(SN, adj, pr, rl, pref, pen, hop_cost=0.0):
    """Dijkstra com sticky replanning. hop_cost (m) é somado a cada aresta para
    penalizar caminhos com mais nós — usar hop_cost > 0 para encurtar corredores."""
    N = len(SN); dist = [1e18]*N; prev = [-1]*N; done = [False]*N
    dist[0] = 0; pq = [(0.0, 0)]; best = 1e18; rp = -1
    while pq:
        d, u = heapq.heappop(pq)
        if done[u]: continue
        done[u] = True
        if rl[u]:
            w = float(np.linalg.norm(SN[u] - pr)) * (1 if u in pref else pen)
            if d + w < best: best = d + w; rp = u
        for v, w0 in adj[u]:
            w = (w0 + hop_cost) * (1 if v in pref else pen)
            if d + w < dist[v]: dist[v] = d + w; prev[v] = u; heapq.heappush(pq, (dist[v], v))
    if rp == -1: return []
    p = []; c = rp
    while c != -1: p.append(c); c = prev[c]
    p = p[::-1]; return p[1:] if p and p[0] == 0 else p

def corredor_lazy(SN, adj, rover_path, CEN, RAD, pen=PEN, hop_cost=0.0):
    """Corredor lazy-Dijkstra com replaneamento sticky. hop_cost > 0 encoraja
    caminhos mais curtos (menos nós) — útil quando n_reles < N_RELES."""
    N = len(SN); adjset = [set(j for j, _ in adj[i]) for i in range(N)]
    prev = None; out = []
    for pr in rover_path:
        rl = [los_radio(SN[i], pr, CEN, RAD) for i in range(N)]; rl[0] = False
        ok = False
        if prev:
            ok = (prev[0] in adjset[0]) and all(prev[i+1] in adjset[prev[i]] for i in range(len(prev)-1)) and rl[prev[-1]]
        if ok: out.append(prev)
        else:
            p = dijkstra_sticky(SN, adj, pr, rl, set(prev) if prev else set(), pen, hop_cost)
            out.append(p); prev = p
    return out

def gerar_trajetoria(P, n_laps=1):
    # PONTOS_ROVER fecha no mesmo ponto -> saltar o 1º ponto das voltas seguintes
    step = ROVER_SPEED * DT
    poli = [np.array(p) for p in P]
    for _ in range(n_laps - 1):
        poli += [np.array(p) for p in P[1:]]
    comp = [np.linalg.norm(poli[i+1]-poli[i]) for i in range(len(poli)-1)]
    acc = [0.0] + list(np.cumsum(comp)); tot = acc[-1]; tr = []; s = 0.0
    while s <= tot:
        idx = 0
        for j in range(len(acc)-1):
            if acc[j] <= s <= acc[j+1]: idx = j; break
        if comp[idx] > 0: tr.append(poli[idx] + (s-acc[idx])/comp[idx] * (poli[idx+1]-poli[idx]))
        s += step
    return tr

# ----- construcao + cache -----
def construir_mapa(stl, cache="mapa_cache.pkl", por_pilar=32, pen=PEN, recalc=False):
    if os.path.exists(cache) and not recalc:
        with open(cache, "rb") as f:
            dados = pickle.load(f)
        # cache válido = 1 volta (rover_path == nf_lap frames)
        if "nf_lap" in dados and len(dados["rover_path"]) == dados["nf_lap"]:
            return dados
        # cache antigo (2 voltas ou sem nf_lap) — reconstruir
    segs = extrair_segmentos_stl(stl)
    CEN, RAD = extrair_pilares(segs)
    SN = np.array([BASE] + gerar_nos(CEN, RAD, por_pilar))
    adj = construir_matriz(SN, CEN, RAD)
    rover_path = gerar_trajetoria(PONTOS_ROVER, n_laps=1)   # 1 volta; loop é cíclico
    nf_lap = len(rover_path)
    coer = corredor_lazy(SN, adj, rover_path, CEN, RAD, pen=pen)
    dados = dict(segs=segs, CEN=CEN, RAD=RAD, SN=SN, adj=adj,
                 rover_path=rover_path, nf_lap=nf_lap, coer=coer)
    with open(cache, "wb") as f: pickle.dump(dados, f)
    return dados

if __name__ == "__main__":
    import time
    sd = os.path.dirname(os.path.abspath(__file__)); stl = os.path.join(sd, "icuas26_1.stl")
    t0 = time.time()
    d = construir_mapa(stl, recalc=True)
    from collections import Counter
    K = Counter(len(c) for c in d["coer"])
    print(f"pilares={len(d['CEN'])}  nos={len(d['SN'])}  frames={len(d['rover_path'])}")
    print(f"distribuicao reles do corredor: {dict(sorted(K.items()))}")
    print(f"tempo={time.time()-t0:.1f}s  cache=mapa_cache.pkl")
