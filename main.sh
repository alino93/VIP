#!/bin/bash

echo "Estimate IMU orientation ... Run Observer"
python3 runExtData.py

echo "Estimate limb angles ... Run LSTM Net"
python3 runNet.py

echo "Estimate position of joints and ZUPT from Geometry ... Run Matlab code"
python3 runMatlab.py

echo "Optimize the results and apply physical constraints ... Run Physics Optimization"
python3 runPhysics.py
