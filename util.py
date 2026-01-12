import torch
from stable_baselines3 import PPO
import os
import socket

class OnnxableSB3Policy(torch.nn.Module):
    def __init__(self, policy):
        super().__init__()
        self.policy = policy

    def forward(self, observation: torch.Tensor):
        # Note: Uses deterministic=True for deterministic actions
        return self.policy(observation, deterministic=True)

def exportONNX(model : PPO, folderName : str):
    # Convert the policy to ONNX format
    onnxable_policy = OnnxableSB3Policy(model.policy)

    # Create a dummy input matching your observation space shape (1, 29)
    dummy_input = torch.randn(1, *model.observation_space.shape) # type: ignore

    try:
        os.mkdir("./models")
        print(f"Directory './models' created successfully.")
    except FileExistsError:
        print(f"Directory './models' already exists.")
    except PermissionError:
        print(f"Permission denied: Unable to create './models'.")
    except Exception as e:
        print(f"An error occurred: {e}")
    
    
    try:
        os.mkdir(f"./models/{folderName}")
        print(f"Directory './models/{folderName}' created successfully.")
    except FileExistsError:
        print(f"Directory './models/{folderName}' already exists.")
    except PermissionError:
        print(f"Permission denied: Unable to create './models/{folderName}'.")
    except Exception as e:
        print(f"An error occurred: {e}")

    model.save(f"./models/{folderName}/continue.zip")
    
    # Export to ONNX
    torch.onnx.export(
        onnxable_policy,
        dummy_input, # type: ignore
        f"./models/{folderName}/model.onnx",  # Output filename
        input_names=["input"], # Input name in ONNX model
        output_names=["output"] # Output name in ONNX model
    )
    
    

def recv_exact(sock, n):
    return sock.recv(n)
