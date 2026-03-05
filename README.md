# Interface Homme-Machine (IHM)

## Objectif

L’IHM a pour objectif :

- d'afficher les différents paramètres de l'intercepteur et du rover (vitesse, position, vue depuis les caméras, etc.) afin que leurs opérateurs puissent agir en conséquence si un évènement imprévu survient.
- de pouvoir armer et désarmer le drone, ainsi qu'interrompre la mission si nécessaire grâce à un bouton d'arrêt d'urgence.
- de pouvoir lancer les missions à distance en étant connecté en **Secure Shell (SSH)** avec le rover ou l’intercepteur.

## Architecture

L'IHM est composée de deux programmes :

1. Un **nœud ROS2** chargé de récupérer les données utiles (position, orientation, mode de guidage, état d’armement de l’intercepteur, etc.) et de les traiter (par exemple en changeant de référentiel les différentes positions).
2. Un programme chargé de **l’affichage graphique**.

## Lancement de l’IHM

1. Se placer dans le dossier : `Interceptor/interceptor_ws`
2. Compiler le package ROS2 correspondant : `colcon build --packages-select ihm_intercepteur `
3. Lancer l’IHM : `ros2 launch ihm_intercepteur ihm.launch.py `

## Visualisation de la mission

L’IHM permet de visualiser :

- la **position et l’orientation du drone intercepteur**
- la **position de la cible**

Ces positions sont affichées sur une carte générée avec **Leaflet**, une bibliothèque JavaScript permettant de créer des cartes interactives.

> N.B.: Cette approche nécessite une **connexion Internet** afin de télécharger les tuiles de carte nécessaires à l’affichage.

### Interaction avec la carte

Afin d'améliorer le confort de l'opérateur, l'IHM permet :

- de **zoomer**
- de **déplacer la carte**

Une **barre d’échelle** est affichée en bas à gauche et s’adapte automatiquement au niveau de zoom.

## Affichage des caméras et détection

L'IHM permet également de visualiser :

- les **images provenant d’une des caméras du drone**
- les **détections réalisées par YOLO**

Les détections sont affichées sous forme de **boîtes englobantes (bounding boxes)**.

Les images apparaissent **en haut à droite de l’écran**.  
Lorsque les images traitées par **YOLO** sont disponibles, elles sont affichées en priorité par rapport aux images brutes.

## Commandes opérateur

L’ensemble des commandes est situé **en haut à gauche de l’interface**.

Ces contrôles permettent notamment :

- de lancer certaines commandes sans passer par un terminal
- de paramétrer différentes options
- de changer le **repère d’affichage** (repère du monde ou repère du robot)

> N.B.: L’affichage dans le repère du robot pas été implémenté.

## Gestion de l’armement

Un bouton indique l’état d’armement du drone :

| État | Couleur | Signification |
|-----|-----|-----|
| ARMEMENT UNKNOWN | Gris | État inconnu |
| ARMED | Rouge | Drone armé |
| DISARMED | Vert | Drone désarmé |

Cliquer sur ce bouton permet **d’armer ou de désarmer le drone** via le nœud ROS2 **mavros**.

> L’affichage du bouton est mis à jour **uniquement après réception de la confirmation ROS2** indiquant que la commande a bien été prise en compte.

## Mode de fonctionnement

Un second bouton affiche le mode du robot :

| Mode | Couleur |
|-----|-----|
| GUIDED MODE | Bleu |
| MANUAL MODE | Vert |
| UNKNOWN MODE | Gris |

Ce bouton permet de **basculer entre mode manuel et mode guidé** via le nœud ROS2 **mavros**.

Comme pour l’armement, l’état n’est mis à jour **qu’après confirmation via un message ROS2**.

## Bouton d'urgence

Le bouton **EMERGENCY** envoie une commande de **désarmement immédiat** au nœud **mavros**.

Ce bouton a été ajouté afin que l’opérateur puisse **arrêter immédiatement le drone en cas de problème**.

Bien que le bouton d’armement permette aussi de désarmer le drone, le bouton **EMERGENCY** est plus **intuitif en situation d’urgence**.

## Lancement de MAVROS

Un menu déroulant **MAVROS** permet de sélectionner :

- le **baudrate**
- le **port**
- la **plateforme** (rover ou intercepteur)

Les ports disponibles sont **scannés automatiquement**.

Cliquer sur **LAUNCH MAVROS** :

- crée une **session `screen`**
- lance le nœud **mavros**

Les messages qui apparaîtraient normalement dans un terminal sont affichés dans une **zone de logs** située en **haut à droite de l’écran**, volontairement similaire à un terminal.
