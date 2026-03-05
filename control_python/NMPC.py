import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button


class NMPCRandomTargetTracker:
    def __init__(self):
        # État du robot traqueur
        self.state = {'x': 50, 'y': 50, 'vx': 0, 'vy': 0}
        self.time = 0
        self.dt = 0.1

        # État de la cible mobile
        self.target_state = {'x': 300, 'y': 300, 'vx': 0, 'vy': 0}

        # Paramètres de mouvement de la cible
        self.target_change_interval = 2.0  # Secondes entre chaque changement
        self.last_change_time = 0
        self.target_max_acceleration = 3.0
        self.target_ax = 0
        self.target_ay = 0

        # Paramètres NMPC
        self.prediction_horizon = 10
        self.control_weight = 0.5
        self.state_weight = 1.0

        # Historique de trajectoire
        self.trajectory = []
        self.target_trajectory = []

        # Animation
        self.is_running = False

    def robot_dynamics(self, state, control):
        # Modèle dynamique du robot (intégrateur double)
        return {
            'x': state['x'] + state['vx'] * self.dt,
            'y': state['y'] + state['vy'] * self.dt,
            'vx': state['vx'] + control['ax'] * self.dt,
            'vy': state['vy'] + control['ay'] * self.dt
        }

    def update_target(self):
        # Mise à jour périodique du mouvement de la cible
        if self.time - self.last_change_time >= self.target_change_interval:
            # Générer une nouvelle accélération aléatoire
            self.target_ax = np.random.uniform(-self.target_max_acceleration,
                                               self.target_max_acceleration)
            self.target_ay = np.random.uniform(-self.target_max_acceleration,
                                               self.target_max_acceleration)
            self.last_change_time = self.time

        # Mettre à jour l'état de la cible
        self.target_state['vx'] += self.target_ax * self.dt
        self.target_state['vy'] += self.target_ay * self.dt

        # Limiter la vitesse maximale
        max_speed = 50
        speed = np.sqrt(self.target_state['vx'] ** 2 + self.target_state['vy'] ** 2)
        if speed > max_speed:
            self.target_state['vx'] = (self.target_state['vx'] / speed) * max_speed
            self.target_state['vy'] = (self.target_state['vy'] / speed) * max_speed

        self.target_state['x'] += self.target_state['vx'] * self.dt
        self.target_state['y'] += self.target_state['vy'] * self.dt

        # Garder la cible dans les limites
        self.target_state['x'] = np.clip(self.target_state['x'], 50, 550)
        self.target_state['y'] = np.clip(self.target_state['y'], 50, 550)

        # Rebondir sur les bords
        if self.target_state['x'] <= 50 or self.target_state['x'] >= 550:
            self.target_state['vx'] *= -0.8
        if self.target_state['y'] <= 50 or self.target_state['y'] >= 550:
            self.target_state['vy'] *= -0.8

    def get_target_reference(self, t):
        # Prédiction simple de la position future de la cible
        # (suppose que la cible continue avec sa vitesse actuelle)
        return {
            'x': self.target_state['x'] + self.target_state['vx'] * (t - self.time),
            'y': self.target_state['y'] + self.target_state['vy'] * (t - self.time),
            'vx': self.target_state['vx'],
            'vy': self.target_state['vy']
        }

    def compute_cost(self, state, control, ref):
        # Fonction de coût (à minimiser) du NMPC
        state_cost = self.state_weight * (
                (state['x'] - ref['x']) ** 2 +
                (state['y'] - ref['y']) ** 2 +
                (state['vx'] - ref['vx']) ** 2 +
                (state['vy'] - ref['vy']) ** 2
        )
        control_cost = self.control_weight * (
                control['ax'] ** 2 +
                control['ay'] ** 2
        )
        return state_cost + control_cost

    def solve_nmpc(self):
        # NMPC simple
        best_control = {'ax': 0, 'ay': 0}
        min_cost = float('inf')

        search_range = 5
        search_step = 1

        for ax in np.arange(-search_range, search_range + search_step, search_step):
            for ay in np.arange(-search_range, search_range + search_step, search_step):
                control = {'ax': ax, 'ay': ay}
                total_cost = 0
                state = self.state.copy()

                for k in range(self.prediction_horizon):
                    ref = self.get_target_reference(self.time + k * self.dt)
                    total_cost += self.compute_cost(state, control, ref)
                    state = self.robot_dynamics(state, control)

                if total_cost < min_cost:
                    min_cost = total_cost
                    best_control = control

        return best_control

    def simulation_step(self):
        # Mettre à jour la cible
        self.update_target()

        # Un pas de simulation pour le robot
        control = self.solve_nmpc()
        self.state = self.robot_dynamics(self.state, control)

        # Enregistrer les trajectoires
        self.trajectory.append((self.state['x'], self.state['y']))
        self.target_trajectory.append((self.target_state['x'], self.target_state['y']))

        if len(self.trajectory) > 500:
            self.trajectory.pop(0)
        if len(self.target_trajectory) > 500:
            self.target_trajectory.pop(0)

        self.time += self.dt

    def reset(self):
        self.state = {'x': 50, 'y': 50, 'vx': 0, 'vy': 0}
        self.target_state = {'x': 300, 'y': 300, 'vx': 0, 'vy': 0}
        self.time = 0
        self.last_change_time = 0
        self.target_ax = 0
        self.target_ay = 0
        self.trajectory = []
        self.target_trajectory = []
        self.is_running = False

    def get_error_distance(self):
        return np.sqrt(
            (self.state['x'] - self.target_state['x']) ** 2 +
            (self.state['y'] - self.target_state['y']) ** 2
        )

    def get_speed(self):
        return np.sqrt(self.state['vx'] ** 2 + self.state['vy'] ** 2)


def main():
    tracker = NMPCRandomTargetTracker()

    # Configuration de la figure
    fig = plt.figure(figsize=(14, 8))
    fig.patch.set_facecolor('#0f172a')

    # Canvas principal
    ax_main = plt.subplot2grid((4, 3), (0, 0), rowspan=4, colspan=2)
    ax_main.set_xlim(0, 600)
    ax_main.set_ylim(0, 600)
    ax_main.set_aspect('equal')
    ax_main.set_facecolor('#1e293b')
    ax_main.tick_params(colors='white')

    # Initialiser les éléments graphiques
    trajectory_line, = ax_main.plot([], [], 'r-', linewidth=1, alpha=0.6, label='Trajectoire intercepteur')
    target_trajectory_line, = ax_main.plot([], [], 'b-', linewidth=1, alpha=0.4, label='Trajectoire cible')
    robot_point, = ax_main.plot([], [], 'ro', markersize=12, label='Intercepteur')
    target_point, = ax_main.plot([], [], 'bo', markersize=10, label='Cible')
    velocity_arrow = None
    target_velocity_arrow = None

    ax_main.legend(loc='upper right', facecolor='#1e293b', edgecolor='white',
                   labelcolor='white', fontsize=9)
    ax_main.set_title('NMPC - Suivi de Cible Mobile Aléatoire',
                      color='white', fontsize=14, weight='bold', pad=10)

    # Zone de texte pour les statistiques
    stats_text = ax_main.text(30, 550, '', color='white', fontsize=10,
                              bbox=dict(boxstyle='round', facecolor='#334155', alpha=0.8))

    # Sliders
    ax_horizon = plt.subplot2grid((4, 3), (0, 2))
    ax_control_weight = plt.subplot2grid((4, 3), (1, 2))
    ax_state_weight = plt.subplot2grid((4, 3), (2, 2))
    ax_change_interval = plt.subplot2grid((4, 3), (3, 2))

    for ax in [ax_horizon, ax_control_weight, ax_state_weight, ax_change_interval]:
        ax.set_facecolor('#1e293b')

    slider_horizon = Slider(ax_horizon, 'Horizon', 5, 40,
                            valinit=tracker.prediction_horizon, valstep=1, color='#3b82f6')
    slider_control = Slider(ax_control_weight, 'Poids contrôle', 0.01, 1.0,
                            valinit=tracker.control_weight, color='#3b82f6')
    slider_state = Slider(ax_state_weight, 'Poids état', 0.1, 5.0,
                          valinit=tracker.state_weight, color='#3b82f6')
    slider_interval = Slider(ax_change_interval, 'Interval changement', 0.5, 5.0,
                             valinit=tracker.target_change_interval, color='#3b82f6')

    for slider in [slider_horizon, slider_control, slider_state, slider_interval]:
        slider.label.set_color('white')
        slider.valtext.set_color('white')

    def update_horizon(val):
        tracker.prediction_horizon = int(val)

    def update_control(val):
        tracker.control_weight = val

    def update_state(val):
        tracker.state_weight = val

    def update_interval(val):
        tracker.target_change_interval = val

    slider_horizon.on_changed(update_horizon)
    slider_control.on_changed(update_control)
    slider_state.on_changed(update_state)
    slider_interval.on_changed(update_interval)

    # Fonction d'animation
    def animate(frame):
        nonlocal velocity_arrow, target_velocity_arrow

        if tracker.is_running:
            tracker.simulation_step()

        # Mettre à jour les trajectoires
        if len(tracker.trajectory) > 0:
            traj_x, traj_y = zip(*tracker.trajectory)
            trajectory_line.set_data(traj_x, traj_y)

        if len(tracker.target_trajectory) > 0:
            target_x, target_y = zip(*tracker.target_trajectory)
            target_trajectory_line.set_data(target_x, target_y)

        # Mettre à jour les points
        robot_point.set_data([tracker.state['x']], [tracker.state['y']])
        target_point.set_data([tracker.target_state['x']], [tracker.target_state['y']])

        # Mettre à jour les flèches de vitesse
        if velocity_arrow is not None:
            velocity_arrow.remove()
        if target_velocity_arrow is not None:
            target_velocity_arrow.remove()

        velocity_arrow = ax_main.arrow(
            tracker.state['x'], tracker.state['y'],
            tracker.state['vx'] * 5, tracker.state['vy'] * 5,
            head_width=8, head_length=8, fc='r', ec='r', alpha=0.7
        )

        target_velocity_arrow = ax_main.arrow(
            tracker.target_state['x'], tracker.target_state['y'],
            tracker.target_state['vx'] * 5, tracker.target_state['vy'] * 5,
            head_width=8, head_length=8, fc='b', ec='b', alpha=0.7
        )

        # Mettre à jour les statistiques
        stats_text.set_text(
            f"Temps: {tracker.time:.1f}s\n"
            f"Distance: {tracker.get_error_distance():.1f}px\n"
            f"Vitesse intercepteur: {tracker.get_speed():.2f}"
        )

        return (trajectory_line, target_trajectory_line, robot_point,
                target_point, velocity_arrow, target_velocity_arrow, stats_text)

    # Boutons
    def toggle_animation(event):
        tracker.is_running = not tracker.is_running
        btn_play.label.set_text('Pause' if tracker.is_running else 'Démarrer')

    def reset_animation(event):
        tracker.reset()
        trajectory_line.set_data([], [])
        target_trajectory_line.set_data([], [])
        btn_play.label.set_text('Démarrer')

    ax_play = plt.axes([0.7, 0.02, 0.12, 0.04])
    ax_reset = plt.axes([0.83, 0.02, 0.12, 0.04])

    btn_play = Button(ax_play, 'Démarrer', color='#3b82f6', hovercolor='#2563eb')
    btn_reset = Button(ax_reset, 'Reset', color='#475569', hovercolor='#334155')

    btn_play.label.set_color('white')
    btn_reset.label.set_color('white')

    btn_play.on_clicked(toggle_animation)
    btn_reset.on_clicked(reset_animation)

    # Animation
    anim = FuncAnimation(fig, animate, interval=50, blit=True, cache_frame_data=False)

    plt.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.08, hspace=0.3, wspace=0.3)
    plt.show()


if __name__ == "__main__":
    main()