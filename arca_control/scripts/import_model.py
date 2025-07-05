import os
import argparse
from arca_sim.robot_model import AVAILABLE_MODELS

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", choices=AVAILABLE_MODELS, help="The model to import")
    args = parser.parse_args()

    print(f"Selected model: {args.model}")
    
    os.chdir(f"models/{args.model}/mjcf")
    os.system("onshape-to-robot .")

