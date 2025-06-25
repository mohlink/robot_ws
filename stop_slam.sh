#!/bin/bash

# Nom de la session tmux utilisée par launch_ros2.sh
session="SLAM"

# Nom du fichier par défaut
default_filename="my_map"

# Nom du fichier fourni en argument ou valeur par défaut
filename=${1:-$default_filename}

# Chemin complet pour sauvegarder la carte
save_path="$HOME/robot_ws/maps/$filename"

# Définir la commande à exécuter avec le sourcing de l'overlay ROS 2
overlay_setup="source ~/robot_ws/install/setup.bash"
command="ros2 run nav2_map_server map_saver_cli -f $save_path"

# Source l'overlay ROS 2
echo "Sourcing ROS 2 overlay: $overlay_setup"
eval $overlay_setup

# Lancer la commande de sauvegarde de la carte
echo "Lancement de la commande : $command"
eval $command

# Attendre que la commande soit terminée
echo "Attente de la fin de la commande de sauvegarde de la carte..."

# Terminer toutes les fenêtres de la session tmux
if tmux has-session -t $session 2>/dev/null; then
  echo "Terminaison de la session tmux : $session"
  tmux kill-session -t $session
else
  echo "La session tmux $session n'existe pas."
fi

echo "Sauvegarde de la carte terminée dans $save_path et toutes les fenêtres tmux fermées."
