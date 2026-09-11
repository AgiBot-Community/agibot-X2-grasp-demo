<p align="center">
  <a href="https://github.com/AgiBot-Community">
    <img src="assets/agibot-community.png" alt="Logo AgiBot Community" width="152">
  </a>
</p>

<h1 align="center">X2 Grasp</h1>

<p align="center">
  <a href="../README.md">简体中文</a> · <a href="README.en.md">English</a> · <strong>Français</strong>
</p>

<p align="center">Démonstration de préhension visuelle sous ROS 2 pour le robot AgiBot X2</p>

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](../LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB.svg)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-00599C.svg)](https://isocpp.org/)

X2 Grasp est un paquet ROS 2 Humble de préhension visuelle pour le robot X2. Il
réunit la détection AprilTag, la reconnaissance visuelle Grounding, la localisation
RGB-D, la cinématique inverse (IK) avec Pinocchio en C++, la commande des bras et
l'orchestration dans un seul paquet `x2_grasp`. Une Action ROS 2 standard fournit
le suivi de progression, le résultat et l'annulation.

> Commencez avec `execute:=false`. N'activez les mouvements réels qu'après avoir
> vérifié les coordonnées cibles, TF, l'IK et les trajectoires, dégagé l'espace de
> travail des bras et confirmé la disponibilité de l'arrêt d'urgence.

## Fonctionnalités

- Un seul paquet ROS 2 pour les interfaces, la perception, l'IK, la commande et l'orchestration.
- API `x2_grasp/action/Grasp` avec progression, résultat et annulation.
- Trois modes : AprilTag, Grounding et arbitrage automatique.
- Messages typés `PerceptionStatus` pour les états de perception.
- Association des horodatages RGB-D, conservation des images de profondeur, conversion TF et nouvelles tentatives en cas d'échec.
- IK Pinocchio avec plusieurs initialisations, planification cartésienne par étapes et protections à l'exécution.
- Moteur IK natif C++17/pybind11, avec moteur Python de comparaison et repli automatique.
- Nœud rclcpp distinct pour publier les commandes des bras et pinces à 50 Hz avec des échéances absolues.
- URDF X2 simplifié inclus pour l'IK de démonstration, sans paquet de description supplémentaire pour les premiers essais.
- Un seul objectif de préhension actif à la fois, avec vérification des demandes d'annulation.
- Même chaîne de perception et de planification en essai sans mouvement et en exécution réelle.

> Grounding utilise l'API distante de vision Volcengine Ark : les images RGB sont
> envoyées par HTTPS au serveur configuré. AprilTag fonctionne localement. Le mode
> automatique peut également appeler l'API si aucun marqueur récent n'est disponible.
> L'URDF inclus est un modèle IK simplifié, sans description complète des collisions,
> de la commande ou de la simulation haute fidélité. Voir les
> [limites de déploiement](REMOTE_API_AND_URDF.md) (en chinois).

## Prérequis

| Composant | Exigence |
| --- | --- |
| Système d'exploitation | Ubuntu 22.04 sur l'architecture du robot cible |
| ROS 2 | Humble |
| SDK du robot | AimDK et `aimdk_msgs` |
| IK | Pinocchio fourni par le dépôt apt de ROS |
| Caméra | RGB, profondeur, CameraInfo et TF de la caméra vers `base_link` |

Utilisez Python 3.10 du système avec les binaires ROS correspondants. Ne remplacez
pas Pinocchio de ROS par un paquet pip : l'ABI Python et l'architecture CPU doivent correspondre.

## Démarrage rapide

### 1. Récupérer le code source

```bash
git clone \
  https://github.com/AgiBot-Community/agibot-X2-grasp-demo.git \
  ~/x2_grasp_ws
cd ~/x2_grasp_ws
```

### 2. Installer les dépendances

Installez AimDK au préalable, puis chargez l'environnement du robot. Adaptez les
chemins si votre espace de travail ou votre SDK se trouve ailleurs.

```bash
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/.aima/env/bashrc
./scripts/install_dependencies.sh
```

Le script installe `ros-humble-pinocchio`, `ros-humble-pybind11-vendor`,
`pybind11-dev`, `libopencv-dev`, `python3-numpy` et `python3-opencv` via apt.
Un utilisateur autre que root doit disposer de `sudo`.

`x2_command_publisher` fait partie de ce paquet. Lorsque CMake trouve `aimdk_msgs`,
il compile le nœud matériel C++, puis colcon l'installe. Le lancement unifié le
démarre et l'utilise avec `command_backend:=auto`. Un environnement de développement
sans `aimdk_msgs` ignore cette cible matérielle ; cela ne valide pas le déploiement sur robot.

### 3. Compiler et vérifier

```bash
colcon build --packages-select x2_grasp --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

ros2 interface show x2_grasp/action/Grasp
ros2 interface show x2_grasp/action/ExecuteCommand
ros2 interface show x2_grasp/msg/PerceptionStatus
/usr/bin/python3 -c \
  'from x2_arm import native_backend_available; print(native_backend_available())'
```

La dernière commande doit afficher `True` pour le moteur IK natif. `auto` utilise
Python si l'extension est indisponible ; `native` impose l'extension et échoue si elle manque.

### 4. Lancer un essai sans mouvement

Dans le terminal A, choisissez un mode. Pour AprilTag, placez le marqueur 36h11
configuré dans le champ de la caméra :

```bash
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=apriltag ik_backend:=auto execute:=false
```

Pour Grounding :

```bash
export ARK_API_KEY='your-api-key'
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=grounding execute:=false
```

Pour l'arbitrage automatique, définissez la même clé et utilisez `mode:=auto`.
Chaque objectif recherche d'abord une cible AprilTag récente, puis demande une
reconnaissance Grounding si nécessaire. La source ne change pas pendant le mouvement
du bras. Le mode de lancement par défaut est `mode:=grounding`.

### 5. Envoyer un objectif de préhension

Dans le terminal B, rechargez ROS, AimDK et l'espace de travail :

```bash
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
source ~/.aima/env/bashrc
source ~/x2_grasp_ws/install/setup.bash

ros2 action send_goal /x2_grasp/grasp x2_grasp/action/Grasp \
  "{target: cup}" --feedback
```

Les cibles par défaut sont `cup` (gobelet en papier), `bread` (pain) et `bottle`
(flacon de médicaments). Conservez ces identifiants configurés dans les commandes,
quelle que soit la langue du guide. En mode AprilTag, le type sélectionne le réglage
de la pince et le son de fin ; il n'identifie pas visuellement l'objet. Un deuxième
objectif simultané est refusé.

## Exemples et annulation

Depuis la racine du dépôt, l'[exemple Python](../example/send_grasp_goal.py) gère la
progression, les résultats, l'annulation temporisée et l'annulation par `Ctrl+C` :

```bash
python3 example/send_grasp_goal.py cup
python3 example/send_grasp_goal.py bottle --cancel-after 5
```

Le client installé offre le même point d'entrée :

```bash
ros2 run x2_grasp grasp_action_client bread
ros2 run x2_grasp grasp_action_client bottle --cancel-after 5
```

L'annulation est coopérative. Elle arrête les commandes suivantes et annule l'Action
interne `ExecuteCommand` pendant l'exécution. Avec `hold_on_stop=true`, le nœud C++
renvoie la dernière position pour la maintenir. L'annulation ne peut pas annuler les
effets des commandes déjà envoyées et ne constitue pas un arrêt d'urgence matériel.
Voir les [exemples détaillés](../example/README.md) (en chinois).

## Exécution sur le robot réel

Vérifiez l'extension IK native, l'installation du nœud de publication, l'étalonnage
de la caméra, la taille du marqueur, l'échelle de profondeur et TF. Testez chaque
cible et chaque mode requis sans mouvement. Les coordonnées doivent être exprimées
en mètres dans `base_link` et appartenir à l'espace atteignable du bras choisi.
Par défaut, `arm_side:=auto` évalue les deux bras ; utilisez `left` ou `right` pour
l'étalonnage. Dégagez l'espace de travail et vérifiez l'arrêt d'urgence ainsi que le
mode de commande AimDK avant de lancer :

```bash
test -x install/x2_grasp/lib/x2_grasp/x2_command_publisher
ros2 launch x2_grasp unified_grasp.launch.py \
  mode:=apriltag ik_backend:=native command_backend:=native execute:=true
```

Commencez par un seul objectif à faible risque. Avec `execute:=false`, la perception
et la planification s'effectuent sans changer le mode du robot ni envoyer de trajectoires.

## Fonctionnement

```text
Objectif de l'Action Grasp
       |
       v
AprilTag / Grounding / arbitrage automatique
       |
       v
Cible dans base_link + état de perception typé
       |
       v
IK Pinocchio C++ et planification cartésienne par étapes
       |
       v
Résultat sans mouvement ou exécution bras/pince via AimDK
```

## Documentation

Le guide principal est disponible en [chinois](../README.md), [anglais](README.en.md)
et [français](README.fr.md). Les références détaillées ci-dessous sont actuellement
en chinois. L'[index de documentation](README.md) permet de naviguer dans le dépôt.

| Document | Contenu |
| --- | --- |
| [Installation et compilation](INSTALLATION.md) | ROS, AimDK, dépendances, compilation et vérification |
| [Utilisation](USAGE.md) | Modes, Actions, annulation et procédure sur robot |
| [Interfaces](INTERFACES.md) | Actions, messages, topics et sens des champs |
| [Configuration](CONFIGURATION.md) | Perception, IK, trajectoires et audio |
| [Architecture](ARCHITECTURE.md) | Responsabilités, threads et flux de données |
| [Performances et migration C++](PERFORMANCE.md) | Périmètre de l'IK native, mesures et conditions de test |
| [Publication des commandes](PUBLISHING.md) | Nœud C++, annulation, watchdog et validation sur robot |
| [API distante et URDF inclus](REMOTE_API_AND_URDF.md) | Transfert de données et limites du modèle simplifié |
| [Dépannage](TROUBLESHOOTING.md) | Installation, TF, perception, IK et exécution |
| [Guide du paquet](../src/x2_grasp/README.md) | Fonctionnement interne et paramètres |

## Structure du dépôt

```text
.
|-- example/                   # Exemples exécutables
|-- docs/                      # Références d'utilisation et de conception
|-- scripts/                   # Installation des dépendances et utilitaires
`-- src/x2_grasp/
    |-- action/                # Actions ROS 2
    |-- msg/                   # Messages typés
    |-- config/                # Configuration unifiée
    |-- launch/                # Fichier de lancement unifié
    |-- x2_grasp/              # Perception et orchestration
    |-- x2_arm/                # Sélection IK, trajectoires et commande matérielle
    |-- x2_common/             # Utilitaires Python communs
    `-- src/                   # RGB-D, IK et publication des commandes en C++
```

## Tests et mesures de performance

```bash
colcon test --packages-select x2_grasp --event-handlers console_direct+
colcon test-result --verbose
```

Tests Python :

```bash
PYTHONPATH=src/x2_grasp python3 -m pytest src/x2_grasp/test
```

Mesures Pinocchio :

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 scripts/benchmark_ik.py --backend python
/usr/bin/python3 scripts/benchmark_ik.py --backend native
```

Le script rapporte la moyenne, la médiane et le P95 pour la construction du solveur,
la correspondance articulations/configuration, la cinématique directe (FK), l'IK de
position, d'orientation, 6D et d'axe, une chaîne de planification à huit points et
l'interpolation des trajectoires. Comparez des moteurs explicitement sélectionnés
avec une compilation Release. La suite complète nécessite ROS 2, AimDK, Pinocchio,
OpenCV et les messages du robot cible. Indiquez les tests réellement exécutés dans
les signalements de problème et les pull requests.

## Configuration et identifiants

Modifiez [unified_grasp.yaml](../src/x2_grasp/config/unified_grasp.yaml) pour configurer
les topics caméra, l'étalonnage, la géométrie de préhension, les types de cible et
l'audio. Grounding lit d'abord `ARK_API_KEY`, puis le fichier privé
`~/.x2_arm/api_key.yaml` contenant `api_key: "your-api-key"`. Ne versionnez jamais de clé réelle.

Pour ajouter une cible, modifiez le catalogue au début du fichier YAML :

```yaml
target_names: [cup, bread, bottle, apple]
target_descriptions: [一次性纸杯, 长条袋装面包, 长条药瓶, 红色苹果]
target_aliases: [paper_cup=cup, medicine_bottle=bottle, fruit=apple]
target_grip_close_positions: [0.10, 0.10, 0.10, 0.25]
target_pcm_paths: [cup.pcm, bread.pcm, bottle.pcm, ""]
default_pcm_path: grasp_complete.pcm
```

Les descriptions chinoises sont des exemples d'instructions au modèle, conservés
à l'identique dans les guides : gobelet jetable en papier, pain emballé, flacon de
médicaments et pomme rouge. Les tableaux des noms, descriptions, positions de pince
et chemins PCM doivent avoir la même longueur et des indices correspondants.
Les alias sont des associations indépendantes `alias=target`. Les positions de pince
vont de `0.0` (fermée) à `1.0` (ouverte) et doivent être étalonnées. Si un son spécifique
est absent ou illisible, `default_pcm_path` est utilisé. Le format audio est PCM S16LE
mono à 16 kHz. Voir la [référence de configuration](CONFIGURATION.md).

## Problèmes courants

- Pinocchio introuvable : installez `ros-humble-pinocchio` et chargez ROS ; ne le remplacez pas par une version pip.
- Type d'Action introuvable : recompilez puis chargez `install/setup.bash` dans le terminal courant.
- Objectif refusé : vérifiez le nom configuré et l'absence d'un autre objectif actif.
- Aucune coordonnée cible : vérifiez les topics caméra, TF et l'étalonnage du marqueur ; pour Grounding, vérifiez aussi la clé, le réseau et la synchronisation RGB-D.
- Essai sans mouvement réussi mais robot immobile : vérifiez `execute`, les services AimDK et le mode de commande en suivant la [procédure d'utilisation](USAGE.md).

Les commandes de diagnostic figurent dans le [guide de dépannage](TROUBLESHOOTING.md).

## Licence

Ce projet est distribué sous [licence Apache 2.0](../LICENSE). ROS, AimDK, les services
de modèles et les ressources du robot tiers restent soumis à leurs propres licences
et conditions. L'attribution du logo figure dans les [ressources de marque](assets/README.md).
