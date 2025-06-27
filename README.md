# Robotics Research Project

This repository demonstrates a simple robotics simulation that combines computer vision and robot control using PyBullet. It includes synthetic data generation, YCB object support, and a trash detection model.

## Project Overview

This project simulates a robotic arm interacting with objects in a virtual environment.

## Project Structure

- `src/main.py`: Main demo combining vision and PyBullet simulation
- `src/conveyor.py`: Conveyor belt simulation
- `src/urdf/load_ycb_objects.py`: Loads YCB or variant objects into PyBullet
- `models/trash_detection_model/`: Placeholder for ML models
- `data/synthetic_dataset/`: Synthetic images, labels, and class list
- `scripts/generate_synthetic_ycb.py`: Generate synthetic dataset
- `scripts/generate_variant_urdfs.py`: Generate URDF variants for YCB objects

## Setup

Install dependencies:

```bash
pip install -r requirements.txt
```
