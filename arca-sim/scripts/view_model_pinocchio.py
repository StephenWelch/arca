import numpy as np
import argparse
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
from arca_sim.robot_model import get_scene_path, get_model_path

def main(args):
    # model_path = get_model_path(args.model)
    # model_path = "/Users/swelch/code/arca/arca-sim/models/reduced/mjcf/robot.xml"
    model_path = "/Users/swelch/code/arca/arca-sim/models/full/mjcf/robot.xml"
    print(f"Loading MJCF model from: {model_path}")
    model, collision_model, visual_model = pin.buildModelsFromMJCF(model_path)
    q0 = pin.neutral(model)

    vis = MeshcatVisualizer(model, collision_model, visual_model)
    vis.initViewer(open=True)
    vis.loadViewerModel()
    while True:
        vis.display(q0)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=str, default="full", choices=["full", "reduced"])
    args = parser.parse_args()
    main(args)