import torch
from classes import OnnxableSB3Policy
from stable_baselines3 import PPO
import os
import datetime
from controller import Field

def exportONNX(model : PPO, robotDef : str, titleField : Field):
    # Convert the policy to ONNX format
    onnxable_policy = OnnxableSB3Policy(model.policy)

    # Create a dummy input matching your observation space shape (1, 29)
    dummy_input = torch.randn(1, *model.observation_space.shape) # type: ignore

    try:
        os.mkdir("../../models")
        print(f"Directory '../../models' created successfully.")
    except FileExistsError:
        print(f"Directory '../../models' already exists.")
    except PermissionError:
        print(f"Permission denied: Unable to create '../../models'.")
    except Exception as e:
        print(f"An error occurred: {e}")
    
    folderName = titleField.getSFString()
    
    try:
        os.mkdir(f"../../models/{folderName}")
        print(f"Directory '../../models/{folderName}' created successfully.")
    except FileExistsError:
        print(f"Directory '../../models/{folderName}' already exists.")
    except PermissionError:
        print(f"Permission denied: Unable to create '../../models/{folderName}'.")
    except Exception as e:
        print(f"An error occurred: {e}")

    # Export to ONNX
    torch.onnx.export(
        onnxable_policy,
        dummy_input, # type: ignore
        f"../../models/{folderName}/ONNX_{robotDef}.onnx",  # Output filename
        input_names=["input"], # Input name in ONNX model
        output_names=["output"] # Output name in ONNX model
    )
    
    model.save(f"../../models/{folderName}/continue.zip")
    
def canImportONNX() -> bool:
    try:
        os.mkdir("../../models")
        print(f"Directory '../../models' created successfully.")
        return False
    except FileExistsError:
        print(f"Directory '../../models' already exists.")
    except PermissionError:
        print(f"Permission denied: Unable to create '../../models'.")
    except Exception as e:
        print(f"An error occurred: {e}")
        
    return os.path.exists("../../models/continue.zip")