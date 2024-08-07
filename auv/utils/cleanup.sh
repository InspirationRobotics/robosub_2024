#!/bin/bash

# List all screen sessions, extract their IDs, and terminate each one
screen -ls | grep -o '[0-9]*\.' | xargs -I {} screen -X -S {} quit

echo "All screens have now been terminated."

sleep 2

sudo shutdown now
