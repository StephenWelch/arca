import mujoco
import os

AVAILABLE_MODELS = os.listdir("models")

def get_mujoco_model(model_type:str)->mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(f"models/{model_type}/mjcf/compiled.xml")
