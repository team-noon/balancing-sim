from stable_baselines3 import PPO

def exportONNX(model: PPO, folderName: str):
    import os
    import torch

    # Eval mode for deterministic tracing
    model.policy.eval()

    # Wrap policy for ONNX
    class OnnxableSB3Policy(torch.nn.Module):
        def __init__(self, policy):
            super().__init__()
            self.policy = policy
        def forward(self, observation: torch.Tensor):
            return self.policy(observation, deterministic=True)

    onnxable_policy = OnnxableSB3Policy(model.policy)

    # Dummy input
    dummy_input = torch.randn(1, *model.observation_space.shape)

    # Create folders
    os.makedirs(f"./models/{folderName}", exist_ok=True)

    # Save original model
    model.save(f"./models/{folderName}/continue.zip")

    # Export ONNX
    torch.onnx.export(
        onnxable_policy,
        dummy_input,  # shape: (1, 198)
        "./models/policy.onnx",
        input_names=["input"],
        output_names=["output"],
        opset_version=18,
        verbose=True
    )

    print(f"ONNX model exported to './models/{folderName}/model.onnx'")

    # Restore train mode
    model.policy.train()
    
    

def recv_exact(sock, n):
    return sock.recv(n)
