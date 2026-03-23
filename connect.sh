#!/bin/bash

echo "Connecting to the simulator..."
ssh -i ./robotic_crew_rsa_gcp -L 5901:localhost:5901 rc@35.232.79.9
