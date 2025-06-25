#!/bin/bash

# Nom du fichier par défaut
default_filename="my_map"

# Nom du fichier fourni en argument ou valeur par défaut
filename=${1:-$default_filename}

# Définir la commande à exécuter avec le sourcing de l'overlay ROS 2
overlay_setup="source ~/robot_ws/install/setup.bash"
command="ros2 run nav2_map_server map_saver_cli -f $filename"

# Source l'overlay ROS 2
# echo "Sourcing ROS 2 overlay: $overlay_setup"
# eval $overlay_setup

# Lancer la commande
echo "save map : $command"
eval $command

