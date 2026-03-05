import numpy as np
import matplotlib.pyplot as plt


class NMPCRandomTargetTracker:
    def __init__(self, seed=None):
        if seed is not None:
            np.random.seed(seed)

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

        # Paramètres NMPC
        self.prediction_horizon = 10
        self.control_weight = 0.5
        self.state_weight = 1.0

    def robot_dynamics(self, state, control):
        return {
            'x': state['x'] + state['vx'] * self.dt,
            'y': state['y'] + state['vy'] * self.dt,
            'vx': state['vx'] + control['ax'] * self.dt,
            'vy': state['vy'] + control['ay'] * self.dt
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

    def get_target_reference(self, t):
        return {
            'x': self.target_state['x'] + self.target_state['vx'] * (t - self.time),
            'y': self.target_state['y'] + self.target_state['vy'] * (t - self.time),
            'vx': self.target_state['vx'],
            'vy': self.target_state['vy']
        }

    def compute_cost(self, state, control, ref):
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

    def get_position_error(self):
        return np.sqrt(
            (self.state['x'] - self.target_state['x']) ** 2 +
            (self.state['y'] - self.target_state['y']) ** 2
        )

    def get_orientation_error(self):
        """
        Erreur d'orientation : angle entre le vecteur vitesse de l'intercepteur
        et la direction vers la cible (en degrés).
        """
        dx = self.target_state['x'] - self.state['x']
        dy = self.target_state['y'] - self.state['y']

        speed = np.sqrt(self.state['vx'] ** 2 + self.state['vy'] ** 2)
        if speed < 1e-6:
            return 0.0

        # Angle de la vitesse du robot
        angle_robot = np.arctan2(self.state['vy'], self.state['vx'])
        # Angle vers la cible
        angle_target = np.arctan2(dy, dx)

        error_rad = angle_target - angle_robot
        # Ramener dans [-pi, pi]
        error_rad = (error_rad + np.pi) % (2 * np.pi) - np.pi
        return np.degrees(error_rad)

    def run(self, tf=15.0):
        """Lance la simulation jusqu'à tf, retourne les séries temporelles."""
        times = []
        pos_errors = []
        orient_errors = []

        steps = int(tf / self.dt)
        for _ in range(steps):
            self.update_target()
            control = self.solve_nmpc()
            self.state = self.robot_dynamics(self.state, control)
            self.time += self.dt

            times.append(self.time)
            pos_errors.append(self.get_position_error())
            orient_errors.append(self.get_orientation_error())

        return np.array(times), np.array(pos_errors), np.array(orient_errors)


def run_all_simulations(n_simulations=5, tf=15.0):
    print(f"Lancement de {n_simulations} simulations (tf = {tf}s)...")

    all_times = []
    all_pos_errors = []
    all_orient_errors = []

    for i in range(n_simulations):
        print(f"  Simulation {i+1}/{n_simulations}...", end=" ", flush=True)
        tracker = NMPCRandomTargetTracker(seed=i * 42)
        times, pos_err, orient_err = tracker.run(tf=tf)
        all_times.append(times)
        all_pos_errors.append(pos_err)
        all_orient_errors.append(orient_err)
        print(f"terminée. Erreur pos finale: {pos_err[-1]:.1f} px, orient: {orient_err[-1]:.1f}°")

    return all_times, all_pos_errors, all_orient_errors


def plot_results(all_times, all_pos_errors, all_orient_errors):
    n = len(all_times)
    colors = plt.cm.tab10(np.linspace(0, 0.5, n))

    fig, axes = plt.subplots(2, 1, figsize=(13, 9))
    fig.patch.set_facecolor('#0f172a')
    fig.suptitle(
        f'NMPC – Résultats de {n} simulations (tf = {all_times[0][-1]:.1f} s)',
        color='white', fontsize=15, fontweight='bold', y=0.98
    )

    # ── Erreur de position ──────────────────────────────────────────────────
    ax1 = axes[0]
    ax1.set_facecolor('#1e293b')
    ax1.tick_params(colors='white')
    ax1.xaxis.label.set_color('white')
    ax1.yaxis.label.set_color('white')
    ax1.title.set_color('white')
    for spine in ax1.spines.values():
        spine.set_edgecolor('#475569')

    pos_matrix = np.vstack(all_pos_errors)
    mean_pos = pos_matrix.mean(axis=0)
    std_pos  = pos_matrix.std(axis=0)

    for i in range(n):
        ax1.plot(all_times[i], all_pos_errors[i],
                 color=colors[i], linewidth=1.2, alpha=0.6, label=f'Sim {i+1}')

    ax1.plot(all_times[0], mean_pos, color='white', linewidth=2.2,
             linestyle='--', label='Moyenne')
    ax1.fill_between(all_times[0], mean_pos - std_pos, mean_pos + std_pos,
                     color='white', alpha=0.10, label='± 1 écart-type')

    ax1.set_title('Erreur de position (distance intercepteur – cible)', color='white', fontsize=12)
    ax1.set_xlabel('Temps (s)', color='white')
    ax1.set_ylabel('Erreur de position (px)', color='white')
    ax1.legend(facecolor='#1e293b', edgecolor='#475569', labelcolor='white', fontsize=9,
               loc='upper right', ncol=2)
    ax1.grid(True, color='#334155', linestyle='--', alpha=0.5)

    # Annotations finales
    for i in range(n):
        ax1.annotate(f'{all_pos_errors[i][-1]:.0f}',
                     xy=(all_times[i][-1], all_pos_errors[i][-1]),
                     xytext=(4, 0), textcoords='offset points',
                     color=colors[i], fontsize=8, va='center')

    # ── Erreur d'orientation ────────────────────────────────────────────────
    ax2 = axes[1]
    ax2.set_facecolor('#1e293b')
    ax2.tick_params(colors='white')
    ax2.xaxis.label.set_color('white')
    ax2.yaxis.label.set_color('white')
    ax2.title.set_color('white')
    for spine in ax2.spines.values():
        spine.set_edgecolor('#475569')

    orient_matrix = np.vstack(all_orient_errors)
    mean_orient = orient_matrix.mean(axis=0)
    std_orient  = orient_matrix.std(axis=0)

    for i in range(n):
        ax2.plot(all_times[i], all_orient_errors[i],
                 color=colors[i], linewidth=1.2, alpha=0.6, label=f'Sim {i+1}')

    ax2.plot(all_times[0], mean_orient, color='white', linewidth=2.2,
             linestyle='--', label='Moyenne')
    ax2.fill_between(all_times[0], mean_orient - std_orient, mean_orient + std_orient,
                     color='white', alpha=0.10, label='± 1 écart-type')

    ax2.axhline(0, color='#64748b', linewidth=0.8, linestyle=':')
    ax2.set_title("Erreur d'orientation (angle vitesse intercepteur → direction cible)", color='white', fontsize=12)
    ax2.set_xlabel('Temps (s)', color='white')
    ax2.set_ylabel("Erreur d'orientation (°)", color='white')
    ax2.legend(facecolor='#1e293b', edgecolor='#475569', labelcolor='white', fontsize=9,
               loc='upper right', ncol=2)
    ax2.grid(True, color='#334155', linestyle='--', alpha=0.5)

    # ── Tableau de statistiques finales ────────────────────────────────────
    stats_lines = ["Statistiques à tf :"]
    for i in range(n):
        stats_lines.append(
            f"  Sim {i+1} → pos: {all_pos_errors[i][-1]:6.1f} px  |  orient: {all_orient_errors[i][-1]:+7.1f}°"
        )
    stats_lines.append(
        f"  Moyenne  → pos: {mean_pos[-1]:6.1f} px  |  orient: {mean_orient[-1]:+7.1f}°"
    )
    fig.text(0.01, 0.01, "\n".join(stats_lines),
             color='#94a3b8', fontsize=8, family='monospace',
             verticalalignment='bottom')

    plt.tight_layout(rect=[0, 0.10, 1, 0.97])
    plt.savefig('nmpc_results.png', dpi=150, bbox_inches='tight',
                facecolor=fig.get_facecolor())
    print("\nGraphique sauvegardé : nmpc_results.png")
    plt.show()


if __name__ == "__main__":
    TF = 15.0
    N_SIMULATIONS = 5

    all_times, all_pos_errors, all_orient_errors = run_all_simulations(
        n_simulations=N_SIMULATIONS, tf=TF
    )
    plot_results(all_times, all_pos_errors, all_orient_errors)