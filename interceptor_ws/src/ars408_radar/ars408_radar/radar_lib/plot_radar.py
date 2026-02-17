
import matplotlib.pyplot as plt
import matplotlib.patches as patches

#simple visualisation of the radar objects
def init_topdown_plot():
    import matplotlib.pyplot as plt

    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlabel("Distance Longitudinale (m)")
    ax.set_ylabel("Distance Latérale (m)")
    ax.set_title("Vue Top-Down des Objets Radar")
    ax.grid(True)
    ax.axis("equal")

    return fig, ax

def plot_objects_topdown(objects, ax):
    import matplotlib.pyplot as plt
    import itertools

    ax.cla()  # on nettoie l’ancienne frame

    # --- Radar en (0,0) ---
    ax.plot(0, 0, "ko")
    ax.text(0, 0, " Radar", fontsize=9, ha="left", va="bottom")

    colors = itertools.cycle(plt.cm.tab10.colors)

    for obj_id, obj_data in objects.items():
        color = next(colors)

        general = obj_data.get("0x60b", {})
        extended = obj_data.get("0x60d", {})

        dist_long = general.get("DistLong")
        dist_lat  = general.get("DistLat")
        width     = extended.get("Width")

        if dist_long is None or dist_lat is None or width is None:
            continue

        # Trait vertical = largeur
        ax.plot(
            [dist_long, dist_long],
            [dist_lat - width/2, dist_lat + width/2],
            color=color,
            linewidth=2
        )

        ax.text(dist_long, dist_lat, str(obj_id),
                color=color, ha="center", va="center", fontsize=9)

    ax.set_xlabel("Distance Longitudinale (m)")
    ax.set_ylabel("Distance Latérale (m)")
    ax.set_title("Vue Top-Down des Objets Radar")
    ax.grid(True)
    ax.axis("equal")

    plt.draw()
    plt.pause(0.001)