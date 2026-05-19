from stable_baselines3 import SAC, sac
from typing import List

def exportModel(model: SAC, folderName: str):
    import os
    import torch

    # Eval mode for deterministic tracing
    model.policy.eval()
    
    model.policy.set_training_mode(False)

    #class OnnxableSB3Policy(torch.nn.Module):
    #    def __init__(self, policy :sac.CnnPolicy):
    #        super().__init__()
    #        
    #        self.features = policy.features_extractor
    #        self.policy_net = policy.mlp_extractor.policy_net
    #        self.action_net = policy.action_net
    #
    #    def forward(self, x):
    #        x = self.features(x)
    #        x = self.policy_net(x)
    #        
    #        return self.action_net(x)
#
    #onnxable_policy = OnnxableSB3Policy(model.policy)
#
    #onnxable_policy.eval()

    # Dummy input
    #dummy_input = torch.randn(1, *model.observation_space.shape)

    # Create folders
    os.makedirs(f"./models/{folderName}", exist_ok=True)

    # Save original model
    model.save(f"./models/{folderName}/continue.zip")

    # Export ONNX
    #torch.onnx.export(
    #    onnxable_policy,
    #    dummy_input,  # shape: (1, 462)
    #    f"./models/{folderName}/model.onnx",
    #    input_names=["input"],
    #    output_names=["output"],
    #    verbose=True,
    #    export_params=True,
    #    external_data=False,
    #    keep_initializers_as_inputs=True
    #)
    
    # Save to header file
    
    
    
    weights : List[float] = []
    
    biases : List[float] = []
    
    inputs : int = model.observation_space.shape.__len__()
    
    outputs : int = model.action_space.shape.__len__()
    
    hiddenLayerSize :int = -1
    hiddenLayerAmount : int=-1
    
    for param in model.policy.named_parameters():
        if(param[0].startswith("actor.") and (param[0].endswith(".weight") or param[0].endswith(".bias")) and not param[0].__contains__("log_std")):
            if param[0].endswith(".weight"):
                weights.extend(param[1].flatten().tolist())
            if param[0].endswith(".bias"):
                if(hiddenLayerSize == -1):
                    hiddenLayerSize = param[1].__len__()
                    hiddenLayerAmount = 0
                
                if(hiddenLayerSize == param[1].__len__()):
                    hiddenLayerAmount += 1
                biases.extend(param[1].flatten().tolist())
                
    if(hiddenLayerSize == outputs):
        hiddenLayerAmount-=1
        
        
            
    headerText : str= f'''
        /**
        * Team noon on top
        */
    
        #pragma once
        
        #include <debug.h>
        
        const struct {"{"}
            u8 ins, outs;
            u8 layers, size;
            
            float bias[{hiddenLayerAmount*hiddenLayerSize + outputs}];
            float weight[{hiddenLayerSize * outputs + hiddenLayerSize*inputs + (max(0,hiddenLayerAmount-1)) *hiddenLayerSize}];
        {"}"} _nn_model = {"{"}
        
            .ins = {inputs},
            
            .outs = {outputs},
            
            .layers = {hiddenLayerAmount},
            
            .size = {hiddenLayerSize},
            
            .bias = {"{"}
                {",".join(map(str, biases))}
            {"}"},
            
            .weight = {"{"}
                {",".join(map(str, weights))}
            {"}"},
        
        
        {"}"};
    
    
    '''
    
    try:
        os.remove(f"./models/{folderName}/model.h")
    except:
        pass
    
    with open(f"./models/{folderName}/model.h", "w") as text_file:
        text_file.write(headerText)

    print(f"model exported to './models/{folderName}'")

    # Restore train mode
    model.policy.train()
    
    model.policy.set_training_mode(True)
    

