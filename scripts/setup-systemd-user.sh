#!/bin/bash

echo "Setting up systemd user service..."

# Create user systemd directory if it doesn't exist
mkdir -p ~/.config/systemd/user

# Copy service file to user systemd directory
cp /home/emackinnon1/InterbotixControlServer/scripts/interbotix-control.service ~/.config/systemd/user/

# Reload systemd user daemon
systemctl --user daemon-reload

# Enable the service to start automatically
systemctl --user enable interbotix-control.service

echo "Systemd user service setup complete"
echo "You can now use: systemctl --user start/stop/restart interbotix-control"
