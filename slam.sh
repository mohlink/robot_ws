#!/bin/bash

# Définir les commandes à exécuter avec le sourcing de l'overlay ROS 2
overlay_setup="source ~/robot_ws/install/setup.bash"

commands=(
  "$overlay_setup && ros2 launch articubot_one launch_robot.launch.py"
  "$overlay_setup && ros2 launch articubot_one rplidar.launch.py"
  "ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/articubot_one/config/mapper_params_online_async.yaml use_sim_time:=false"
)

# Définir les délais (en secondes) entre les lancements de commandes
delays=(10 5 1)  # Spécifiez un délai pour chaque commande respectivement

# Nom de la session tmux
session="SLAM"

# Vérifier si la session existe déjà et la tuer si nécessaire
if tmux has-session -t $session 2>/dev/null; then
  tmux kill-session -t $session
fi

# Créer une nouvelle session tmux détachée
tmux new-session -d -s $session -n "window_1"

# Fonction pour exécuter une commande dans une fenêtre tmux
run_command_in_tmux() {
  local command=$1
  local window=$2
  echo "Lancement $window "
  tmux send-keys -t $session:$window "$command" C-m
}

# Exécuter les commandes séquentiellement dans de nouvelles fenêtres avec des délais différents
for i in "${!commands[@]}"; do
  window_name="window_$((i+1))"
  if [ $i -eq 0 ]; then
    # Renommer et exécuter la première commande dans la première fenêtre
    tmux rename-window -t $session:0 "$window_name"
    run_command_in_tmux "${commands[$i]}" "window_1"
  else
    # Attendre le délai spécifique avant de lancer la commande suivante
    echo "Delais ${delays[$((i-1))]} s"
    sleep ${delays[$((i-1))]}
    tmux new-window -t $session -n "$window_name"
    run_command_in_tmux "${commands[$i]}" "$window_name"
  fi
done

# Attacher à la session tmux pour permettre la gestion manuelle des fenêtres
tmux attach-session -t $session
