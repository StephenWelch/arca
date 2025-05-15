# Arca Sim

Control code and simulator for the Arca biped.

## Getting started
- [Import a Model](#importing-a-model)
- [Start the simulator](#starting-the-simulation)

## Starting the simulation
Run `uv run scripts/sim.py --model <model_type>`, where `model_type` is a directory under `models`

## Importing a Model
Model paths follow this format: `models/<model_type>/<urdf|mjcf>`. 
- Each subfolder should have its own `config.json` for `onshape-to-robot`. 

Download the model into its corresponding path by running `uv run scripts/import_model.py --model <model_type>`. 

The available model types are:
- `full`: Models all linkages in the robot model
- `reduced`: Models only the pin joints of the robot
