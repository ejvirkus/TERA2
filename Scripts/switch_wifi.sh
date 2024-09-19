#!/bin/bash

# List all available Wi-Fi networks and filter by those that are connectable (saved in NetworkManager)
connectable_networks=$(nmcli --fields SSID,SECURITY dev wifi list | awk '$2 != "--" {print $1}')

# Initialize variables to store the strongest network
strongest_signal=-100
best_network=""

# Loop through connectable networks and check their signal strength
for network in $connectable_networks; do
    # Get the signal strength of the current network
    signal_strength=$(nmcli --fields SSID,SIGNAL dev wifi list | grep "$network" | awk '{print $2}')
    
    # Check if this network has a stronger signal than the current strongest
    if [ "$signal_strength" -gt "$strongest_signal" ]; then
        strongest_signal=$signal_strength
        best_network=$network
    fi
done

# Connect to the strongest network
if [ -n "$best_network" ]; then
    nmcli dev wifi connect "$best_network"
    echo "Connected to the strongest connectable network: $best_network with signal strength: $strongest_signal"
else
    echo "No connectable networks found"
fi

