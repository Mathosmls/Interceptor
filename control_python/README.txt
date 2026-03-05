# Simulation de suivi de cible mobile — PID vs NMPC

Ce dossier contient des simulations de poursuite d'une cible mobile par un intercepteur, comparant deux approches de contrôle : **PID** et **NMPC** (Commande Prédictive Non Linéaire).

---

## Fichiers

### `PID.py`
Simulation interactive du contrôleur PID. Une fenêtre animée s'ouvre avec :
- L'intercepteur (rouge) qui poursuit une cible mobile aléatoire (bleu)
- Des sliders pour ajuster en temps réel les gains Kp, Ki, Kd, la vitesse max et l'intervalle de changement de direction de la cible
- Des boutons Démarrer / Reset

### `PID_graphes.py`
Lance 5 simulations PID avec des trajectoires aléatoires différentes (graines fixes pour reproductibilité) et génère un graphique comparatif (`pid_results.png`) avec :
- L'erreur de position au fil du temps (+ moyenne et écart-type)
- L'erreur d'orientation au fil du temps

### `NMPC.py`
Même principe que `PID.py` mais avec un contrôleur NMPC. À chaque pas de temps, le contrôleur optimise une séquence de commandes sur un horizon de prédiction pour minimiser l'écart à la cible. Les sliders permettent de régler l'horizon de prédiction, les poids d'état et de contrôle, et l'intervalle de changement de la cible.

### `NMPC_graphes.py`
Équivalent de `PID_graphes.py` pour le NMPC. Lance 5 simulations et sauvegarde le graphique dans `nmpc_results.png`.

### `plot_Unity_traj.py`
Script utilitaire pour visualiser des trajectoires exportées depuis **Unity**. Il lit un fichier CSV (`trajectory_data.csv`) contenant les positions de deux objets (cible et intercepteur) et trace leurs trajectoires en vue de dessus (plan X/Z). Gère le format décimal français (virgule comme séparateur).

---

## Dépendances

```
numpy
matplotlib
pandas
```

Installation : `pip install numpy matplotlib pandas`