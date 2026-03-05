import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button


class PIDRandomTargetTracker:
    def __init__(self):
        # État du robot traqueur
        self.state = {'x': 50, 'y': 50, 'vx': 0, 'vy': 0}
        self.time = 0
        self.dt = 0.1

        # État de la cible mobile
        self.target_state = {'x': 300, 'y': 300, 'vx': 0, 'vy': 0}

        # Paramètres de mouvement de la cible
        self.target_change_interval = 2.0
        self.last_change_time = 0
        self.target_max_acceleration = 3.0
        self.target_ax = 0
        self.target_ay = 0

        # Paramètres PID
        self.Kp = 1.0
        self.Ki = 0.05
        self.Kd = 0.5

        # Variables internes PID
        self._integral_x = 0.0
        self._integral_y = 0.0
        self._prev_error_x = 0.0
        self._prev_error_y = 0.0

        # Contraintes physiques de l'intercepteur
        self.max_accel = 5.0
        self.max_speed = 20.0   # Vitesse maximale de l'intercepteur

        # Historique de trajectoire
        self.trajectory = []
        self.target_trajectory = []

        # Animation
        self.is_running = False

    def robot_dynamics(self, state, control):
        # Modèle dynamique du robot (intégrateur double)
        new_vx = state['vx'] + control['ax'] * self.dt
        new_vy = state['vy'] + control['ay'] * self.dt

        # Limiter la vitesse maximale de l'intercepteur
        speed = np.sqrt(new_vx ** 2 + new_vy ** 2)
        if speed > self.max_speed:
            new_vx = (new_vx / speed) * self.max_speed
            new_vy = (new_vy / speed) * self.max_speed

        return {
            'x': state['x'] + new_vx * self.dt,
            'y': state['y'] + new_vy * self.dt,
            'vx': new_vx,
            'vy': new_vy
        }

    def update_target(self):
        if self.time - self.last_change_time >= self.target_change_interval:
            self.target_ax = np.random.uniform(-self.target_max_acceleration,
                                               self.target_max_acceleration)
            self.target_ay = np.random.uniform(-self.target_max_acceleration,
                                               self.target_max_acceleration)
            self.last_change_time = self.time

        self.target_state['vx'] += self.target_ax * self.dt
        self.target_state['vy'] += self.target_ay * self.dt

        max_speed = 50
        speed = np.sqrt(self.target_state['vx'] ** 2 + self.target_state['vy'] ** 2)
        if speed > max_speed:
            self.target_state['vx'] = (self.target_state['vx'] / speed) * max_speed
            self.target_state['vy'] = (self.target_state['vy'] / speed) * max_speed

        self.target_state['x'] += self.target_state['vx'] * self.dt
        self.target_state['y'] += self.target_state['vy'] * self.dt

        self.target_state['x'] = np.clip(self.target_state['x'], 50, 550)
        self.target_state['y'] = np.clip(self.target_state['y'], 50, 550)

        if self.target_state['x'] <= 50 or self.target_state['x'] >= 550:
            self.target_state['vx'] *= -0.8
        if self.target_state['y'] <= 50 or self.target_state['y'] >= 550:
            self.target_state['vy'] *= -0.8

    def solve_pid(self):
        error_x = self.target_state['x'] - self.state['x']
        error_y = self.target_state['y'] - self.state['y']

        self._integral_x += error_x * self.dt
        self._integral_y += error_y * self.dt
        integral_limit = 200.0
        self._integral_x = np.clip(self._integral_x, -integral_limit, integral_limit)
        self._integral_y = np.clip(self._integral_y, -integral_limit, integral_limit)

        derivative_x = (error_x - self._prev_error_x) / self.dt
        derivative_y = (error_y - self._prev_error_y) / self.dt

        ax = self.Kp * error_x + self.Ki * self._integral_x + self.Kd * derivative_x
        ay = self.Kp * error_y + self.Ki * self._integral_y + self.Kd * derivative_y

        ax = np.clip(ax, -self.max_accel, self.max_accel)
        ay = np.clip(ay, -self.max_accel, self.max_accel)

        self._prev_error_x = error_x
        self._prev_error_y = error_y

        return {'ax': ax, 'ay': ay}

    def simulation_step(self):
        self.update_target()
        control = self.solve_pid()
        self.state = self.robot_dynamics(self.state, control)

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
        self._integral_x = 0.0
        self._integral_y = 0.0
        self._prev_error_x = 0.0
        self._prev_error_y = 0.0
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
    tracker = PIDRandomTargetTracker()

    # Configuration de la figure
    fig = plt.figure(figsize=(14, 8))
    fig.patch.set_facecolor('#0f172a')

    # Canvas principal
    ax_main = plt.subplot2grid((5, 3), (0, 0), rowspan=5, colspan=2)
    ax_main.set_xlim(0, 600)
    ax_main.set_ylim(0, 600)
    ax_main.set_aspect('equal')
    ax_main.set_facecolor('#1e293b')
    ax_main.tick_params(colors='white')

    trajectory_line, = ax_main.plot([], [], 'r-', linewidth=1, alpha=0.6, label='Trajectoire intercepteur')
    target_trajectory_line, = ax_main.plot([], [], 'b-', linewidth=1, alpha=0.4, label='Trajectoire cible')
    robot_point, = ax_main.plot([], [], 'ro', markersize=12, label='Intercepteur')
    target_point, = ax_main.plot([], [], 'bo', markersize=10, label='Cible')
    velocity_arrow = None
    target_velocity_arrow = None

    ax_main.legend(loc='upper right', facecolor='#1e293b', edgecolor='white',
                   labelcolor='white', fontsize=9)
    ax_main.set_title('PID - Suivi de Cible Mobile Aléatoire',
                      color='white', fontsize=14, weight='bold', pad=10)

    stats_text = ax_main.text(30, 550, '', color='white', fontsize=10,
                              bbox=dict(boxstyle='round', facecolor='#334155', alpha=0.8))

    # Sliders
    ax_kp              = plt.subplot2grid((5, 3), (0, 2))
    ax_ki              = plt.subplot2grid((5, 3), (1, 2))
    ax_kd              = plt.subplot2grid((5, 3), (2, 2))
    ax_max_speed       = plt.subplot2grid((5, 3), (3, 2))
    ax_change_interval = plt.subplot2grid((5, 3), (4, 2))

    for ax in [ax_kp, ax_ki, ax_kd, ax_max_speed, ax_change_interval]:
        ax.set_facecolor('#1e293b')

    slider_kp        = Slider(ax_kp,              'Gain Kp',             0.01, 5.0,
                              valinit=tracker.Kp,             color='#3b82f6')
    slider_ki        = Slider(ax_ki,              'Gain Ki',             0.0,  1.0,
                              valinit=tracker.Ki,             color='#3b82f6')
    slider_kd        = Slider(ax_kd,              'Gain Kd',             0.0,  5.0,
                              valinit=tracker.Kd,             color='#3b82f6')
    slider_max_speed = Slider(ax_max_speed,       'Vitesse max',         5.0, 50.0,
                              valinit=tracker.max_speed,      color='#3b82f6')
    slider_interval  = Slider(ax_change_interval, 'Interval changement', 0.5,  5.0,
                              valinit=tracker.target_change_interval, color='#3b82f6')

    for slider in [slider_kp, slider_ki, slider_kd, slider_max_speed, slider_interval]:
        slider.label.set_color('white')
        slider.valtext.set_color('white')

    def update_kp(val):        tracker.Kp = val
    def update_ki(val):        tracker.Ki = val
    def update_kd(val):        tracker.Kd = val
    def update_max_speed(val): tracker.max_speed = val
    def update_interval(val):  tracker.target_change_interval = val

    slider_kp.on_changed(update_kp)
    slider_ki.on_changed(update_ki)
    slider_kd.on_changed(update_kd)
    slider_max_speed.on_changed(update_max_speed)
    slider_interval.on_changed(update_interval)

    def animate(frame):
        nonlocal velocity_arrow, target_velocity_arrow

        if tracker.is_running:
            tracker.simulation_step()

        if len(tracker.trajectory) > 0:
            traj_x, traj_y = zip(*tracker.trajectory)
            trajectory_line.set_data(traj_x, traj_y)

        if len(tracker.target_trajectory) > 0:
            target_x, target_y = zip(*tracker.target_trajectory)
            target_trajectory_line.set_data(target_x, target_y)

        robot_point.set_data([tracker.state['x']], [tracker.state['y']])
        target_point.set_data([tracker.target_state['x']], [tracker.target_state['y']])

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

        stats_text.set_text(
            f"Temps: {tracker.time:.1f}s\n"
            f"Distance: {tracker.get_error_distance():.1f}px\n"
            f"Vitesse intercepteur: {tracker.get_speed():.2f}"
        )

        return (trajectory_line, target_trajectory_line, robot_point,
                target_point, velocity_arrow, target_velocity_arrow, stats_text)

    def toggle_animation(event):
        tracker.is_running = not tracker.is_running
        btn_play.label.set_text('Pause' if tracker.is_running else 'Démarrer')

    def reset_animation(event):
        tracker.reset()
        trajectory_line.set_data([], [])
        target_trajectory_line.set_data([], [])
        btn_play.label.set_text('Démarrer')

    ax_play  = plt.axes([0.7,  0.02, 0.12, 0.04])
    ax_reset = plt.axes([0.83, 0.02, 0.12, 0.04])

    btn_play  = Button(ax_play,  'Démarrer', color='#3b82f6', hovercolor='#2563eb')
    btn_reset = Button(ax_reset, 'Reset',    color='#475569', hovercolor='#334155')

    btn_play.label.set_color('white')
    btn_reset.label.set_color('white')

    btn_play.on_clicked(toggle_animation)
    btn_reset.on_clicked(reset_animation)

    anim = FuncAnimation(fig, animate, interval=50, blit=True, cache_frame_data=False)

    plt.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.08, hspace=0.3, wspace=0.3)
    plt.show()


if __name__ == "__main__":
    main()