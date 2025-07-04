from abc import ABC, abstractmethod
import mujoco
import os
from xml import etree
from pathlib import Path

AVAILABLE_MODELS = os.listdir("models")
MODEL_DIR = Path("models")

def get_model_path(model_type:str)->Path:
    if model_type not in AVAILABLE_MODELS:
        raise ValueError(f"Invalid model type. Available models: {AVAILABLE_MODELS}")
    return MODEL_DIR / model_type / "mjcf" / "robot.xml"

def get_scene_path(model_type:str)->Path:
    if model_type not in AVAILABLE_MODELS:
        raise ValueError(f"Invalid model type. Available models: {AVAILABLE_MODELS}")
    return MODEL_DIR / model_type / "mjcf" / "scene.xml"