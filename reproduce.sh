#!/usr/bin/env bash
# Reproduces reference_results/ (about 5 min for the experiment on one core).
set -e
python calibrate_minerva.py motor_o --apogee 10000
python -m rocketfusion.characterize --preset minerva --out results/sensor_characterisation
python -m rocketfusion.experiment --preset minerva --protocol held_out --n-train 120 --n-val 40 --n-test 30 --out results/minerva_held_out
