import matplotlib.pyplot as plt
import pandas as pd
import sys, os

CSV_PATH = "trajectory_data.csv"

if not os.path.exists(CSV_PATH):
    print(f"Fichier introuvable : {CSV_PATH}")
    sys.exit(1)

# ── Parsing robuste : Unity (locale FR) écrit les décimales avec une virgule,
#    ce qui multiplie le nombre de colonnes. On regroupe les tokens par paires.
def parse_line(tokens):
    vals = []
    i = 0
    while i < len(tokens):
        if i + 1 < len(tokens):
            try:
                vals.append(float(tokens[i] + '.' + tokens[i+1]))
                i += 2
            except ValueError:
                vals.append(float(tokens[i]))
                i += 1
        else:
            vals.append(float(tokens[i]))
            i += 1
    return vals

with open(CSV_PATH, encoding="utf-8") as f:
    lines = f.read().strip().split('\n')

rows = [parse_line(line.split(',')) for line in lines[1:]]
df = pd.DataFrame(rows, columns=['time', 'obj1_x', 'obj1_z', 'obj2_x', 'obj2_z'])

x1, z1 = df["obj1_x"].values, df["obj1_z"].values
x2, z2 = df["obj2_x"].values, df["obj2_z"].values

# ── Tracé ──────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(10, 8))
fig.patch.set_facecolor('#0f172a')
ax.set_facecolor('#1e293b')
ax.tick_params(colors='white')
ax.set_xlabel('X (m)', color='white')
ax.set_ylabel('Z (m)', color='white')
ax.set_title('Trajectoires comparées — Vue de dessus (X/Z)',
             color='white', fontsize=14, weight='bold', pad=10)
for spine in ax.spines.values():
    spine.set_edgecolor('#475569')

ax.plot(x1, z1, color='#ef4444', linewidth=1.5, label='Cible')
ax.plot(x2, z2, color='#3b82f6', linewidth=1.5, label='Intercepteur')

# Cercle = départ, étoile = arrivée
ax.plot(x1[0],  z1[0],  'o', color='#ef4444', markersize=8)
ax.plot(x1[-1], z1[-1], '*', color='#ef4444', markersize=14)
ax.plot(x2[0],  z2[0],  'o', color='#3b82f6', markersize=8)
ax.plot(x2[-1], z2[-1], '*', color='#3b82f6', markersize=14)

ax.legend(loc='upper right', facecolor='#1e293b', edgecolor='white',
          labelcolor='white', fontsize=10)

plt.tight_layout()
plt.show()