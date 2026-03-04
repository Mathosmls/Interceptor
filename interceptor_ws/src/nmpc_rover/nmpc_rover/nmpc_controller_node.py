#!/usr/bin/env python3
"""
Node ROS2 : Contrôleur NMPC pour rover ArduPilot réel
- Reçoit l'état du rover via /mavros/... topics
- Suit une trajectoire sinusoïdale (points waypoints)
- Publie des commandes de vitesse /mavros/setpoint_velocity/cmd_vel
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Bool
from visualization_msgs.msg import Marker, MarkerArray
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

from scipy.optimize import minimize
from dataclasses import dataclass, field
from typing import Tuple, List
import threading


# ──────────────────────────────────────────────
#  Structures de données
# ──────────────────────────────────────────────

@dataclass
class RoverState:
    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    yaw: float = 0.0
    omega: float = 0.0  # vitesse angulaire en z
    timestamp: float = 0.0

    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.vx, self.vy, self.yaw, self.omega])


@dataclass
class Waypoint:
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0


# ──────────────────────────────────────────────
#  Générateur de trajectoire sinusoïdale
# ──────────────────────────────────────────────

class SinusoidalTrajectory:
    """
    Génère une succession de waypoints formant une sinusoïde.
    Le rover suivra le waypoint le plus proche devant lui.
    """

    def __init__(self,
                 amplitude: float = 3.0,    # m
                 wavelength: float = 10.0,  # m
                 forward_speed: float = 1.0, # m/s (vitesse d'avance nominale)
                 total_length: float = 50.0, # m de progression en X
                 num_points: int = 200):

        self.amplitude = amplitude
        self.wavelength = wavelength
        self.forward_speed = forward_speed
        self.waypoints: List[Waypoint] = []
        self._generate(total_length, num_points)

    def _generate(self, total_length: float, num_points: int):
        xs = np.linspace(0.0, total_length, num_points)
        for x in xs:
            y = self.amplitude * np.sin(2 * np.pi * x / self.wavelength)
            # Dérivée pour obtenir la direction tangentielle
            dydx = self.amplitude * (2 * np.pi / self.wavelength) * np.cos(2 * np.pi * x / self.wavelength)
            heading = np.arctan2(dydx, 1.0)
            vx = self.forward_speed * np.cos(heading)
            vy = self.forward_speed * np.sin(heading)
            self.waypoints.append(Waypoint(x=x, y=y, vx=vx, vy=vy))

    def get_target(self, rover_state: RoverState, lookahead: float = 2.0) -> Waypoint:
        """Retourne le prochain waypoint à lookahead mètres devant le rover."""
        best_wp = self.waypoints[-1]
        min_dist = float('inf')

        # Cherche le waypoint non encore dépassé le plus proche
        for wp in self.waypoints:
            dx = wp.x - rover_state.x
            dy = wp.y - rover_state.y
            dist = np.sqrt(dx**2 + dy**2)
            # On préfère les points devant le rover (projection sur l'axe de cap)
            forward = dx * np.cos(rover_state.yaw) + dy * np.sin(rover_state.yaw)
            if forward > 0 and dist < min_dist:
                min_dist = dist
                best_wp = wp

        # Cherche le point à "lookahead" devant le point le plus proche
        idx = self.waypoints.index(best_wp)
        for wp in self.waypoints[idx:]:
            dx = wp.x - rover_state.x
            dy = wp.y - rover_state.y
            dist = np.sqrt(dx**2 + dy**2)
            if dist >= lookahead:
                return wp

        return self.waypoints[-1]


# ──────────────────────────────────────────────
#  Contrôleur NMPC
# ──────────────────────────────────────────────

class NMPCController:

    def __init__(self,
                 prediction_horizon: int = 10,
                 dt: float = 0.1,
                 control_weight: float = 0.05,
                 state_weight: float = 1.0,
                 heading_weight: float = 5.0,
                 max_linear_vel: float = 1.5,   # m/s
                 max_angular_vel: float = 1.0): # rad/s

        self.N = prediction_horizon
        self.dt = dt
        self.Q_state = state_weight
        self.Q_heading = heading_weight
        self.R = control_weight
        self.max_v = max_linear_vel
        self.max_w = max_angular_vel

        # Paramètres dynamiques rover (modèle unicycle simplifié)
        self.linear_drag = 1.5
        self.angular_drag = 3.0

    def predict_dynamics(self, state: np.ndarray, v: float, w: float) -> np.ndarray:
        """
        Modèle cinématique unicycle :
        state = [x, y, vx, vy, yaw, omega]
        u = [v (vitesse linéaire), w (vitesse angulaire)]
        """
        x, y, vx, vy, yaw, omega = state

        # Dynamique simplifiée avec friction
        ax = v * np.cos(yaw) / self.dt - vx - self.linear_drag * vx
        ay = v * np.sin(yaw) / self.dt - vy - self.linear_drag * vy
        alpha = (w - omega) / self.dt - self.angular_drag * omega

        x_next   = x + vx * self.dt
        y_next   = y + vy * self.dt
        vx_next  = vx + ax * self.dt
        vy_next  = vy + ay * self.dt
        yaw_next = yaw + omega * self.dt
        omega_next = omega + alpha * self.dt

        return np.array([x_next, y_next, vx_next, vy_next, yaw_next, omega_next])

    def cost_function(self, u_flat: np.ndarray,
                      initial_state: np.ndarray,
                      target: Waypoint) -> float:
        total_cost = 0.0
        state = initial_state.copy()
        u_seq = u_flat.reshape((self.N, 2))  # (v, w) pairs

        for k in range(self.N):
            v, w = u_seq[k]
            state = self.predict_dynamics(state, v, w)
            x, y, vx, vy, yaw, omega = state

            # Cible propagée dans le temps
            tx = target.x + target.vx * k * self.dt
            ty = target.y + target.vy * k * self.dt

            pos_err = (x - tx)**2 + (y - ty)**2
            vel_err = (vx - target.vx)**2 + (vy - target.vy)**2

            dx, dy = tx - x, ty - y
            desired_yaw = np.arctan2(dy, dx)
            heading_err = self._angle_diff(yaw, desired_yaw)**2

            state_cost   = self.Q_state * (pos_err + 0.1 * vel_err)
            state_cost  += self.Q_heading * heading_err
            control_cost = self.R * (v**2 + w**2)

            total_cost += state_cost + control_cost

        return total_cost

    def _angle_diff(self, a1: float, a2: float) -> float:
        d = a1 - a2
        return (d + np.pi) % (2 * np.pi) - np.pi

    def solve(self, state: RoverState, target: Waypoint) -> Tuple[float, float]:
        """Retourne (v, w) : vitesse linéaire et angulaire optimales."""
        init = state.as_array()
        u0 = np.zeros(self.N * 2)

        bounds = []
        for _ in range(self.N):
            bounds.append((-self.max_v, self.max_v))   # v
            bounds.append((-self.max_w, self.max_w))   # w

        result = minimize(
            fun=self.cost_function,
            x0=u0,
            args=(init, target),
            method='SLSQP',
            bounds=bounds,
            options={'maxiter': 50, 'disp': False, 'ftol': 1e-4}
        )

        u_opt = result.x.reshape((self.N, 2))
        return float(u_opt[0, 0]), float(u_opt[0, 1])


# ──────────────────────────────────────────────
#  Node ROS2 principal
# ──────────────────────────────────────────────

class NMPCControllerNode(Node):

    def __init__(self):
        super().__init__('nmpc_controller')

        # ── Paramètres ROS ──
        self.declare_parameter('prediction_horizon', 10)
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('control_weight', 0.05)
        self.declare_parameter('state_weight', 1.0)
        self.declare_parameter('heading_weight', 5.0)
        self.declare_parameter('max_linear_vel', 1.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('traj_amplitude', 3.0)
        self.declare_parameter('traj_wavelength', 10.0)
        self.declare_parameter('traj_forward_speed', 1.0)
        self.declare_parameter('traj_total_length', 50.0)
        self.declare_parameter('control_rate', 10.0)  # Hz

        p = lambda name: self.get_parameter(name).value

        # ── Contrôleur ──
        self.controller = NMPCController(
            prediction_horizon=p('prediction_horizon'),
            dt=p('dt'),
            control_weight=p('control_weight'),
            state_weight=p('state_weight'),
            heading_weight=p('heading_weight'),
            max_linear_vel=p('max_linear_vel'),
            max_angular_vel=p('max_angular_vel'),
        )

        # ── Trajectoire ──
        self.trajectory = SinusoidalTrajectory(
            amplitude=p('traj_amplitude'),
            wavelength=p('traj_wavelength'),
            forward_speed=p('traj_forward_speed'),
            total_length=p('traj_total_length'),
        )

        # ── État ──
        self.rover_state = RoverState()
        self.state_received = False
        self.mavros_connected = False
        self.armed = False
        self._lock = threading.Lock()

        # ── QoS ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── Subscribers ──
        self.create_subscription(Odometry, '/mavros/local_position/odom',
                                 self._odom_callback, sensor_qos)
        self.create_subscription(State, '/mavros/state',
                                 self._state_callback, 10)

        # ── Publishers ──
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.target_marker_pub = self.create_publisher(
            MarkerArray, '/nmpc/trajectory_markers', 10)
        self.debug_pub = self.create_publisher(
            Float64MultiArray, '/nmpc/debug', 10)

        # ── Services ArduPilot ──
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # ── Timer boucle de contrôle ──
        rate = p('control_rate')
        self.create_timer(1.0 / rate, self._control_loop)

        # ── Timer publication trajectoire (visualisation Rviz) ──
        self.create_timer(2.0, self._publish_trajectory_markers)

        self.get_logger().info('✅ NMPC Controller Node démarré')
        self.get_logger().info(f'   Trajectoire sinusoïdale: A={p("traj_amplitude")}m, '
                               f'λ={p("traj_wavelength")}m')

    # ────────────────────────────────────────
    #  Callbacks
    # ────────────────────────────────────────

    def _odom_callback(self, msg: Odometry):
        with self._lock:
            pos = msg.pose.pose.position
            vel = msg.twist.twist.linear
            q   = msg.pose.pose.orientation
            ang = msg.twist.twist.angular

            self.rover_state.x  = pos.x
            self.rover_state.y  = pos.y
            self.rover_state.vx = vel.x
            self.rover_state.vy = vel.y
            self.rover_state.omega = ang.z
            # Yaw depuis quaternion
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y**2 + q.z**2)
            self.rover_state.yaw = np.arctan2(siny_cosp, cosy_cosp)
            self.rover_state.timestamp = self.get_clock().now().nanoseconds * 1e-9
            self.state_received = True

    def _state_callback(self, msg: State):
        self.mavros_connected = msg.connected
        self.armed = msg.armed

    # ────────────────────────────────────────
    #  Boucle principale
    # ────────────────────────────────────────

    def _control_loop(self):
        if not self.state_received:
            self.get_logger().warn('En attente de télémétrie...', throttle_duration_sec=5.0)
            return

        if not self.mavros_connected:
            self.get_logger().warn('MAVROS non connecté', throttle_duration_sec=5.0)
            return

        with self._lock:
            state_snapshot = RoverState(**self.rover_state.__dict__)

        lookahead = self.get_parameter('lookahead_distance').value
        target = self.trajectory.get_target(state_snapshot, lookahead=lookahead)

        # ── Calcul NMPC ──
        v_opt, w_opt = self.controller.solve(state_snapshot, target)

        # ── Envoi commande ──
        self._send_velocity_command(v_opt, w_opt)

        # ── Debug ──
        dist = np.sqrt((state_snapshot.x - target.x)**2 +
                       (state_snapshot.y - target.y)**2)
        heading_err = np.degrees(self.controller._angle_diff(
            state_snapshot.yaw,
            np.arctan2(target.y - state_snapshot.y, target.x - state_snapshot.x)
        ))

        debug_msg = Float64MultiArray()
        debug_msg.data = [
            state_snapshot.x, state_snapshot.y,
            target.x, target.y,
            v_opt, w_opt,
            dist, heading_err
        ]
        self.debug_pub.publish(debug_msg)

        self.get_logger().debug(
            f'pos=({state_snapshot.x:.2f},{state_snapshot.y:.2f}) '
            f'target=({target.x:.2f},{target.y:.2f}) '
            f'dist={dist:.2f}m err_h={heading_err:.1f}° '
            f'v={v_opt:.3f} w={w_opt:.3f}',
        )

    def _send_velocity_command(self, v: float, w: float):
        """Publie une commande vitesse vers MAVROS (mode GUIDED)."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Rover : vitesse linéaire en X, angulaire en Z
        msg.twist.linear.x  = float(v)
        msg.twist.linear.y  = 0.0
        msg.twist.linear.z  = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = float(w)

        self.cmd_vel_pub.publish(msg)

    # ────────────────────────────────────────
    #  Visualisation Rviz
    # ────────────────────────────────────────

    def _publish_trajectory_markers(self):
        markers = MarkerArray()

        # Ligne de la trajectoire
        line = Marker()
        line.header.frame_id = 'map'
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = 'sinusoid'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.05
        line.color.r = 0.0
        line.color.g = 0.8
        line.color.b = 1.0
        line.color.a = 1.0

        from geometry_msgs.msg import Point
        for wp in self.trajectory.waypoints:
            p = Point()
            p.x, p.y, p.z = wp.x, wp.y, 0.0
            line.points.append(p)

        markers.markers.append(line)

        # Cible courante
        with self._lock:
            state_snap = RoverState(**self.rover_state.__dict__)
        target = self.trajectory.get_target(
            state_snap, self.get_parameter('lookahead_distance').value)

        sphere = Marker()
        sphere.header.frame_id = 'map'
        sphere.header.stamp = self.get_clock().now().to_msg()
        sphere.ns = 'target'
        sphere.id = 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = target.x
        sphere.pose.position.y = target.y
        sphere.pose.position.z = 0.1
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.4
        sphere.color.r = 1.0
        sphere.color.g = 0.3
        sphere.color.b = 0.0
        sphere.color.a = 1.0
        markers.markers.append(sphere)

        self.target_marker_pub.publish(markers)


# ──────────────────────────────────────────────
#  Entry point
# ──────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = NMPCControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
