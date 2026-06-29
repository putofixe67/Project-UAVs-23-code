#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Planeador feedforward com PROCURA POR UNIAO numa janela.
Substitui o frágil eff=argmax(K): o alvo de cada frame é o conjunto de nós
distintos do corredor em [t, t+L] (nó atual U futuros), espalhados por arcos
distintos. Durante um salto de nó, um relé segura o antigo e outro ocupa o novo
(bracketing). Movimento com limite ESTRITO <= STEP (sem teletransporte)."""
import numpy as np
from scipy.optimize import linear_sum_assignment
from mapa import (BASE, N_RELES, V_MAX, D_MIN, STEP, DT,
                  los_radio, corredor_nav)

# ----- routing por comprimento de arco (mover anda AO LONGO do corredor) -----
def _arclen(P):
    P = [np.asarray(p, float) for p in P]; acc = [0.0]
    for i in range(len(P)-1): acc.append(acc[-1] + np.linalg.norm(P[i+1] - P[i]))
    return P, acc

def _ponto(P, acc, s):
    s = float(np.clip(s, 0, acc[-1]))
    for i in range(len(acc)-1):
        if acc[i] <= s <= acc[i+1]:
            seg = acc[i+1] - acc[i]; f = 0 if seg < 1e-12 else (s - acc[i]) / seg
            return P[i] + f * (P[i+1] - P[i])
    return P[-1]

def _proj(P, acc, p):
    p = np.asarray(p, float); mb = 1e18; sb = 0.0
    for i in range(len(P)-1):
        a = P[i]; b = P[i+1]; d = b - a; L2 = d.dot(d)
        if L2 < 1e-12: continue
        t = np.clip((p - a).dot(d) / L2, 0, 1); e = np.linalg.norm(p - (a + t*d))
        if e < mb: mb = e; sb = acc[i] + t*np.sqrt(L2)
    return sb

def mover(p, tgt_arc, P, acc, CEN, RAD):
    """Avanca <=STEP ao longo da polilinha; nunca corta pilares (com margem nav)."""
    s0 = _proj(P, acc, p); sg = s0 + np.clip(tgt_arc - s0, -STEP, STEP); w = _ponto(P, acc, sg)
    d = w - p; di = np.linalg.norm(d)
    if di < 1e-9: return p.copy()
    cand = p + d/di * min(di, STEP)
    if corredor_nav(p, cand, CEN, RAD): return cand
    best = None; mb = 1e18                       # bloqueado: vertice nav-livre mais perto do destino
    for v in P:
        v = np.asarray(v, float)
        if corredor_nav(p, v, CEN, RAD):
            dd = np.linalg.norm(v - w)
            if dd < mb: mb = dd; best = v
    if best is not None:
        d2 = best - p; di2 = np.linalg.norm(d2)
        if di2 > 1e-9: return p + d2/di2 * min(di2, STEP)
    return p.copy()

def separar(pos, d_min=D_MIN, iters=6):
    pos = [p.copy() for p in pos]
    for _ in range(iters):
        for i in range(len(pos)):
            for j in range(i+1, len(pos)):
                d = pos[j] - pos[i]; dist = np.linalg.norm(d)
                if dist < 1e-9: d = np.array([np.cos(i+j), np.sin(i+j)]); dist = 1.0
                if dist < d_min:
                    e = (d_min - dist) / 2; u = d / dist; pos[i] -= u*e; pos[j] += u*e
    return pos

def resolver(prev, alvo, CEN, RAD, folga_min, iters=60, pontos_fixos=None):
    """Projecao iterada (Gauss-Seidel): separacao D_MIN entre relés, folga de
    pilares, e afastamento de pontos fixos (e.g. drone sombra) que nao podem
    ser movidos. O cap de velocidade é a ultima operacao → vmax<=1.5 garantido."""
    pos = [p.copy() for p in alvo]
    for _ in range(iters):
        pos = separar(pos, D_MIN, iters=1)
        if pontos_fixos:
            for i in range(len(pos)):
                for fp in pontos_fixos:
                    d = pos[i] - np.asarray(fp, float); dist = np.linalg.norm(d)
                    if dist < D_MIN:
                        u = d / dist if dist > 1e-9 else np.array([1.0, 0.0])
                        pos[i] += u * (D_MIN - dist)
        pos = folga_pilares(pos, CEN, RAD, folga_min)
        for i in range(len(pos)):
            net = pos[i] - prev[i]; nd = np.linalg.norm(net)
            if nd > STEP: pos[i] = prev[i] + net/nd*STEP
    return pos

def folga_pilares(pos, CEN, RAD, folga_min=0.30):
    """Empurrao radial: garante que cada relé fica a >=folga_min da superficie do
    pilar mais proximo. Mantém os drones bem afastados sem impor a margem total."""
    out = []
    for p in pos:
        p = p.copy(); dc = np.linalg.norm(CEN - p, axis=1); k = int(np.argmin(dc - RAD))
        folga = dc[k] - RAD[k]
        if folga < folga_min:
            u = (p - CEN[k]) / (dc[k] + 1e-12); p = CEN[k] + u * (RAD[k] + folga_min)
        out.append(p)
    return out

def _atribuir(dpos, alvos, k_fundo, n):
    """Atribui relés a alvos: o nó-fundo (vê o rover) primeiro ao relé mais perto,
    Hungarian no resto. Devolve {relé: k_alvo}."""
    fundo_pt = alvos[k_fundo][0]
    r_fundo = int(np.argmin([np.linalg.norm(dpos[i] - fundo_pt) for i in range(n)]))
    reles_rest = [i for i in range(n) if i != r_fundo]
    alvos_rest = [k for k in range(len(alvos)) if k != k_fundo]
    C = np.zeros((len(reles_rest), len(alvos_rest)))
    for a, i in enumerate(reles_rest):
        for b, k in enumerate(alvos_rest):
            C[a, b] = np.linalg.norm(dpos[i] - alvos[k][0])
    ri, ci = linear_sum_assignment(C)
    par = {r_fundo: k_fundo}
    for a, b in zip(ri, ci): par[reles_rest[a]] = alvos_rest[b]
    return par

def planear(coer, SN, rover_path, CEN, RAD, L, folga_min=0.30, init_pos=None):
    nf = len(rover_path)
    if init_pos is not None:
        dpos = [np.asarray(p, float).copy() for p in init_pos]
    else:
        ang = np.linspace(0, 2*np.pi, N_RELES, endpoint=False)
        dpos = [BASE + 0.45*np.array([np.cos(a), np.sin(a)]) for a in ang]
    FD = []; FR = []; des = []
    for t in range(nf):
        prev = [p.copy() for p in dpos]; dpos = [p.copy() for p in dpos]
        alvos, k_fundo = alvos_uniao(coer, t, L, SN, N_RELES, CEN=CEN, RAD=RAD)
        par = _atribuir(dpos, alvos, k_fundo, N_RELES)
        novo = [p.copy() for p in dpos]
        for i, k in par.items():
            _, P, acc, arc = alvos[k]
            novo[i] = mover(dpos[i], arc, P, acc, CEN, RAD)
        dpos = resolver(prev, novo, CEN, RAD, folga_min)
        nos, lig, ligado = rede_comunicacao(BASE, dpos, rover_path[t], CEN, RAD)
        FD.append([p.copy() for p in dpos]); FR.append((nos, lig, ligado))
        if not ligado: des.append(t)
    return FD, FR, des

def planear_sombra(coer, SN, rover_path, CEN, RAD, L, n_reles=4, folga_min=0.30,
                   init_pos=None, init_sombra=None):
    """1 drone-SOMBRA exatamente por cima do rover + n_reles relés livres a ligar
    base->sombra. O sombra garante sempre um nó na posicao do rover (regra: a
    ligacao ao rover e feita por um drone). Indice do sombra = ultimo em FD[t]."""
    nf = len(rover_path)
    if init_pos is not None:
        dpos = [np.asarray(p, float).copy() for p in init_pos]
    else:
        ang = np.linspace(0, 2*np.pi, n_reles, endpoint=False)
        dpos = [BASE + 0.45*np.array([np.cos(a), np.sin(a)]) for a in ang]
    sombra = np.asarray(init_sombra if init_sombra is not None else rover_path[0], float)
    FD = []; FR = []; des = []
    for t in range(nf):
        prev = [p.copy() for p in dpos]; dpos = [p.copy() for p in dpos]
        # sombra segue o rover (limite estrito; rover<<vmax -> fica exato)
        alvo_s = np.asarray(rover_path[t], float); ds = alvo_s - sombra; nds = np.linalg.norm(ds)
        sombra = alvo_s if nds <= STEP else sombra + ds/nds*STEP
        # relés com procura por uniao
        alvos, k_fundo = alvos_uniao(coer, t, L, SN, n_reles, CEN=CEN, RAD=RAD)
        par = _atribuir(dpos, alvos, k_fundo, n_reles)
        novo = [p.copy() for p in dpos]
        for i, k in par.items():
            _, P, acc, arc = alvos[k]
            novo[i] = mover(dpos[i], arc, P, acc, CEN, RAD)
        dpos = resolver(prev, novo, CEN, RAD, folga_min, pontos_fixos=[sombra])
        # rede: drones = relés + sombra (o sombra esta no rover -> ve-o sempre)
        drones = [p.copy() for p in dpos] + [sombra.copy()]
        nos, lig, ligado = rede_comunicacao(BASE, drones, rover_path[t], CEN, RAD)
        FD.append(drones); FR.append((nos, lig, ligado))
        if not ligado: des.append(t)
    return FD, FR, des

def rede_comunicacao(base, drones, rover, CEN, RAD):
    """Conetividade {base, drones, rover}. REGRA: sem aresta direta base<->rover."""
    nos = [np.asarray(base)] + [np.asarray(d) for d in drones] + [np.asarray(rover)]
    M = len(nos); adj = {i: [] for i in range(M)}; lig = []
    for i in range(M):
        for j in range(i+1, M):
            if i == 0 and j == M-1: continue
            if los_radio(nos[i], nos[j], CEN, RAD):
                adj[i].append(j); adj[j].append(i); lig.append((i, j))
    vis = [False]*M; st = [0]; vis[0] = True
    while st:
        u = st.pop()
        for v in adj[u]:
            if not vis[v]: vis[v] = True; st.append(v)
    return nos, lig, all(vis)

# ----- nucleo: alvos por UNIAO na janela -----
def _polilinha(idx_cadeia, SN, min_len=0.0):
    """Polilinha navegável BASE -> nós da cadeia.
    Se min_len > 0 e o corredor for mais curto, estende na direcção do último
    segmento (em frente ao fundo, na direcção do rover) para que os drones
    extra tenham targets espaçados a >= D_MIN."""
    P = [np.asarray(BASE, float)] + [np.asarray(SN[i], float) for i in idx_cadeia]
    if len(P) == 1: P = [P[0], P[0] + np.array([0.4, 0.0])]
    P, acc = _arclen(P)
    if min_len > 0 and acc[-1] < min_len:
        ext = P[-1] - P[-2]; enorm = np.linalg.norm(ext)
        ext = ext / enorm if enorm > 1e-9 else np.array([1.0, 0.0])
        P = list(P) + [P[-1] + ext * (min_len - acc[-1])]
        P, acc = _arclen(P)
    return P, acc

def alvos_uniao(coer, t, L, SN, n_reles=N_RELES, CEN=None, RAD=None):
    """Targets uniformemente espaçados em PA + bracketing na polilinha futura PC.
    Uniform fill garante targets >= D_MIN*1.1 entre si. Bracketing pré-posiciona
    1 drone na direcção do próximo nó fundo para 0 disconnections em transições.
    _atribuir usa projecção em PA para ordenar todos os targets → sem cruzamentos."""
    nf = len(coer)
    janela = [coer[(t + s) % nf] for s in range(L + 1)]
    A = coer[t % nf]
    min_len = n_reles * D_MIN * 1.1
    PA, accA = _polilinha(A, SN, min_len=min_len)

    if len(A) > n_reles:
        # corredor mais longo que os relés: distribuição uniforme directa
        alvos = []
        for i in range(n_reles):
            s = accA[-1] * (i + 1) / n_reles
            alvos.append((_ponto(PA, accA, s), PA, accA, s))
        return alvos, n_reles - 1

    # 1) nós da cadeia actual
    alvos = []
    for k, idx in enumerate(A, start=1):
        alvos.append((np.asarray(SN[idx], float), PA, accA, accA[k]))
    # 2) bracketing: 1 drone no próximo nó fundo (usa PC para movimento correcto)
    vistos = set(A)
    for C in janela[1:]:
        if not C: continue
        fundo = C[-1]
        if fundo in vistos: continue
        vistos.add(fundo)
        PC, accC = _polilinha(C, SN, min_len=min_len)
        alvos.append((np.asarray(SN[fundo], float), PC, accC, accC[-1]))
        if len(alvos) >= n_reles: break
    k_fundo = len(A) - 1 if A else 0
    # 3) slots restantes: uniform fill em PA, garantindo >= D_MIN de todos os existentes
    if len(alvos) < n_reles:
        existing_pts = [pos for (pos, _, _, _) in alvos]
        for i in range(n_reles * 4):          # iterações suficientes para encontrar slots
            if len(alvos) >= n_reles: break
            s = accA[-1] * (i + 1) / (n_reles * 4)
            pt = _ponto(PA, accA, s)
            if all(np.linalg.norm(pt - ep) >= D_MIN for ep in existing_pts):
                alvos.append((pt, PA, accA, s))
                existing_pts.append(pt)
    return alvos[:n_reles], k_fundo

def suavizar(FD, alpha=0.82):
    """Filtro EMA passa-baixo: elimina vibração frame-a-frame sem alterar a
    trajetória global. alpha=0 → só posição anterior; alpha=1 → sem suavização.
    EMA só reduz variação entre frames, logo v_max e clearance só melhoram."""
    out = [[p.copy() for p in FD[0]]]
    for t in range(1, len(FD)):
        out.append([alpha * FD[t][i] + (1 - alpha) * out[-1][i]
                    for i in range(len(FD[t]))])
    return out

def metricas(FD, des, nf):
    vmax = max(np.linalg.norm(FD[t][i]-FD[t-1][i])
               for t in range(1, len(FD)) for i in range(N_RELES)) / DT
    return len(des), vmax

if __name__ == "__main__":
    import os, time, pickle
    sd = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(sd, "mapa_cache.pkl"), "rb") as f: d = pickle.load(f)
    SN, CEN, RAD, rover_path, coer = d["SN"], d["CEN"], d["RAD"], d["rover_path"], d["coer"]
    nf = len(rover_path)
    print(f"frames={nf}  (sweep de LEAD)")
    for L in (150, 250, 350, 500):
        t0 = time.time()
        FD, FR, des = planear(coer, SN, rover_path, CEN, RAD, L)
        nd, vmax = metricas(FD, des, nf)
        print(f"  L={L:4d}  desligados={nd:4d}  vmax={vmax:.3f} m/s  ({time.time()-t0:.0f}s)")
