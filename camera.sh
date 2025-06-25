#!/bin/bash

# Définir la commande à exécuter avec le sourcing de l'overlay ROS 2
overlay_setup="source ~/robot_ws/install/setup.bash"
command="ros2 launch articubot_one camera.launch.py"

# Source l'overlay ROS 2
eval $overlay_setup

# Lancer la commande
eval $command
