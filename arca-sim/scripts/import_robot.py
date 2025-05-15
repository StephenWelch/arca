import os
import argparse

if __name__ == "__main__":
    available_models = os.listdir("models")
    
    parser = argparse.ArgumentParser()
    parser.add_argument("model", choices=available_models, help="The model to import")
    args = parser.parse_args()

    print(f"Selected model: {args.model}")
    
    os.chdir(f"models/{args.model}/mjcf")
    os.system("onshape-to-robot .")

