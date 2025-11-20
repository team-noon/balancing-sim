import torch
from classes import OnnxableSB3Policy
from stable_baselines3 import PPO
import os
import datetime

def exportONNX(model : PPO, robotDef : str):
    # Convert the policy to ONNX format
    onnxable_policy = OnnxableSB3Policy(model.policy)

    # Create a dummy input matching your observation space shape (1, 29)
    dummy_input = torch.randn(1, *model.observation_space.shape)

    try:
        os.mkdir("../../models")
        print(f"Directory '../../models' created successfully.")
    except FileExistsError:
        print(f"Directory '../../models' already exists.")
    except PermissionError:
        print(f"Permission denied: Unable to create '../../models'.")
    except Exception as e:
        print(f"An error occurred: {e}")

    time = datetime.datetime.now()
    
    try:
        os.mkdir(f"../../models/{time}")
        print(f"Directory '../../models/{time}' created successfully.")
    except FileExistsError:
        print(f"Directory '../../models/{time}' already exists.")
    except PermissionError:
        print(f"Permission denied: Unable to create '../../models/{time}'.")
    except Exception as e:
        print(f"An error occurred: {e}")

    # Export to ONNX
    torch.onnx.export(
        onnxable_policy,
        dummy_input,
        f"../../models/{time}/ONNX_{robotDef}.onnx",  # Output filename
        input_names=["input"], # Input name in ONNX model
        output_names=["output"] # Output name in ONNX model
    )