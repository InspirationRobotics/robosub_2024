#!/bin/bash

# Check if there are any screen sessions running
if screen -list | grep -q "\.pts"; then
  echo "Found screen sessions. Terminating..."

  # Loop through each screen session and kill it
  screen -ls | grep "\.pts" | awk '{print $1}' | while read -r session_id; do
    screen -S "$session_id" -X quit
    echo "Terminated screen session: $session_id"
  done

  echo "All screen sessions have been terminated."
else
  echo "No screen sessions found."
fi
