#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Nó ROS2 — rede de relay de drones (ICUAS26 / Gazebo).

Reproduz a trajetoria feedforward validada offline:

  modo=sombra  (default): 4 relés seguem setpoints pré-calculados +
               1 drone-SOMBRA segue a pose do rover em tempo real.

  modo=livre : 5 relés livres seguem todos setpoints pré-calculados.

CHAVE: os relés NÃO são indexados pelo tempo absoluto, mas pelo PROGRESSO do
rover real na rota conhecida (projeção da pose do rover na polilinha -> arco ->
frame). Assim, se o Gazebo correr o rover mais devagar/depressa, a cadeia
mantém-se sincronizada com o rover.

Interface assumida: Crazyswarm2 (https://imrclab.github.io/crazyswarm2/).
Publica crazyflie_interfaces/msg/Position em /<prefixo><i>/cmd_position e usa
o serviço Takeoff. Confirma os nomes de tópicos/serviços no teu ambiente.
"""
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point
try:
    from crazyflie_interfaces.msg import Position
    from crazyflie_interfaces.srv import Takeoff
    TEM_CF = True
except Exception:
    TEM_CF = False


def _proj_arco(rota, acc, p):
    """Projeta p na polilinha 'rota' e devolve o comprimento de arco s."""
    melhor_e, s = 1e18, 0.0
    for i in range(len(rota) - 1):
        a, b = rota[i], rota[i + 1]; d = b - a; L2 = float(d @ d)
        if L2 < 1e-12:
            continue
        t = max(0.0, min(1.0, float((p - a) @ d) / L2))
        proj = a + t * d; e = float(np.linalg.norm(p - proj))
        if e < melhor_e:
            melhor_e = e; s = acc[i] + t * np.sqrt(L2)
    return s


class RelayChainNode(Node):
    def __init__(self):
        super().__init__("relay_chain_node")
        # ---- parâmetros ----
        self.declare_parameter("traj_path", "trajetoria_relay.npz")
        self.declare_parameter("rover_odom_topic", "/AGV/pose")
        self.declare_parameter("drone_prefix", "cf_")      # cf_1..cf_5
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("takeoff_height", 1.0)
        self.declare_parameter("takeoff_duration", 3.0)
        self.declare_parameter("control_yaw", 0.0)
        self.declare_parameter("modo", "sombra")           # "sombra" ou "livre"

        self.modo = self.get_parameter("modo").get_parameter_value().string_value
        if self.modo not in ("sombra", "livre"):
            raise ValueError(f"modo deve ser 'sombra' ou 'livre', recebeu '{self.modo}'")

        td = np.load(self.get_parameter("traj_path").get_parameter_value().string_value)
        self.reles = td["reles_xy"]            # [T, n_reles, 2]
        self.rota  = td["rota_xy"].astype(float)
        self.acc   = td["acc"].astype(float)
        self.n_reles   = int(td["n_reles"])
        self.alt_voo   = float(td["alt_voo"])
        self.alt_sombra = float(td.get("alt_sombra", self.alt_voo - 0.2))
        self.rover_speed = float(td["rover_speed"])
        self.sim_fps     = float(td["sim_fps"])
        self.step_arco   = self.rover_speed / self.sim_fps
        self.T     = self.reles.shape[0]
        self.v_max = float(td["v_max"])

        self.rover_xy = None
        prefix  = self.get_parameter("drone_prefix").get_parameter_value().string_value
        n_total = self.n_reles + (1 if self.modo == "sombra" else 0)
        self.nomes    = [f"{prefix}{i+1}" for i in range(n_total)]
        self.i_sombra = self.n_reles if self.modo == "sombra" else None

        if not TEM_CF:
            self.get_logger().warn("crazyflie_interfaces não encontrado — modo simulado "
                                   "(loga setpoints em vez de publicar).")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.pubs = {}
        if TEM_CF:
            for nm in self.nomes:
                self.pubs[nm] = self.create_publisher(Position, f"/{nm}/cmd_position", 10)

        self.create_subscription(Point,
            self.get_parameter("rover_odom_topic").get_parameter_value().string_value,
            self._cb_rover, qos)

        self.ultimo = {nm: None for nm in self.nomes}
        self.dt = 1.0 / self.get_parameter("rate_hz").get_parameter_value().double_value
        self._takeoff_pedido = False
        self.timer = self.create_timer(self.dt, self._ciclo)
        self.get_logger().info(
            f"relay_chain_node pronto: modo={self.modo}, "
            f"{self.n_reles} relés{' + 1 sombra' if self.modo == 'sombra' else ''}, T={self.T}")

    def _cb_rover(self, msg: Point):
        self.rover_xy = np.array([msg.x, msg.y], float)

    def _takeoff(self):
        if not TEM_CF:
            return
        h   = self.get_parameter("takeoff_height").get_parameter_value().double_value
        dur = self.get_parameter("takeoff_duration").get_parameter_value().double_value
        for nm in self.nomes:
            cli = self.create_client(Takeoff, f"/{nm}/takeoff")
            if cli.wait_for_service(timeout_sec=2.0):
                req = Takeoff.Request()
                req.height = h
                req.duration = rclpy.duration.Duration(seconds=dur).to_msg()
                cli.call_async(req)
            else:
                self.get_logger().warn(f"serviço takeoff de {nm} indisponível")

    def _saturar(self, nm, alvo_xy):
        ant = self.ultimo[nm]
        if ant is None:
            self.ultimo[nm] = alvo_xy.copy(); return alvo_xy
        d  = alvo_xy - ant; nd = float(np.linalg.norm(d)); lim = self.v_max * self.dt
        novo = alvo_xy if nd <= lim else ant + d / nd * lim
        self.ultimo[nm] = novo.copy(); return novo

    def _publicar(self, nm, xy, z):
        if not TEM_CF:
            self.get_logger().info(f"{nm} -> ({xy[0]:.2f}, {xy[1]:.2f}, {z:.2f})")
            return
        m = Position()
        m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = "world"
        m.x, m.y, m.z = float(xy[0]), float(xy[1]), float(z)
        m.yaw = self.get_parameter("control_yaw").get_parameter_value().double_value
        self.pubs[nm].publish(m)

    def _ciclo(self):
        if self.rover_xy is None:
            return
        if not self._takeoff_pedido:
            self._takeoff(); self._takeoff_pedido = True; return
        s = _proj_arco(self.rota, self.acc, self.rover_xy)
        t = int(np.clip(round(s / self.step_arco), 0, self.T - 1))
        for i in range(self.n_reles):
            nm = self.nomes[i]
            xy = self._saturar(nm, self.reles[t, i])
            self._publicar(nm, xy, self.alt_voo)
        if self.modo == "sombra":
            nm = self.nomes[self.i_sombra]
            xy = self._saturar(nm, self.rover_xy.copy())
            self._publicar(nm, xy, self.alt_sombra)


def main(args=None):
    rclpy.init(args=args)
    node = RelayChainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
