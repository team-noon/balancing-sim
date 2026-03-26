from stable_baselines3 import PPO

def exportONNX(model: PPO, folderName: str):
    import os
    import torch

    # Eval mode for deterministic tracing
    model.policy.eval()
    
    model.policy.set_training_mode(False)

    class OnnxableSB3Policy(torch.nn.Module):
        def __init__(self, policy):
            super().__init__()
            self.features = policy.features_extractor
            self.policy_net = policy.mlp_extractor.policy_net
            self.action_net = policy.action_net
    
        def forward(self, x):
            x = self.features(x)
            x = self.policy_net(x)
            return self.action_net(x)

    onnxable_policy = OnnxableSB3Policy(model.policy)

    onnxable_policy.eval()

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
        f"./models/{folderName}/model.onnx",
        input_names=["input"],
        output_names=["output"],
        verbose=True,
        export_params=True,
        external_data=False,
        keep_initializers_as_inputs=True
    )

    print(f"ONNX model exported to './models/{folderName}/model.onnx'")

    # Restore train mode
    model.policy.train()
    
    model.policy.set_training_mode(True)
    
    

def recv_exact(sock, n):
    return sock.recv(n)
