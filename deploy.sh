#!/bin/bash

echo "Copying files to the simulator VM..."
rsync -avz -e "ssh -i ./robotic_crew_rsa_gcp -o StrictHostKeyChecking=no" --exclude 'deploy.sh' --exclude '.git' ./ rc@35.232.79.9:~/inorbit_g1_sim/