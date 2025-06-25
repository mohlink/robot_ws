#!/bin/bash

# Nom de la session tmux utilisée par auto_nav.sh
session="AUTO_NAV"

# Terminer toutes les fenêtres de la session tmux
if tmux has-session -t $session 2>/dev/null; then
  echo "Terminaison de la session tmux : $session"
  tmux kill-session -t $session
  echo "Toutes les fenêtres tmux associées à la session $session ont été fermées."
else
  echo "La session tmux $session n'existe pas."
fi
