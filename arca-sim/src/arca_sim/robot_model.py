from typing import Literal

def get_scene_path(model_type: Literal["full", "reduced"]) -> str:
    return f"models/{model_type}/mjcf/scene.xml"
    
def get_model_path(model_type: Literal["full", "reduced"]) -> str:
    return f"models/{model_type}/mjcf/robot.xml"
