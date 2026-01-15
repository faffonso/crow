
import os
import torch
import torchvision.models as models
import numpy as np

def load_model(runid):
    print(f"Loading model for runid: {runid}")
    model = models.mobilenet_v2()
    # Modify first layer as in RTInference.py
    model.features[0][0] = torch.nn.Conv2d(1, 32, kernel_size=3, stride=2, padding=1, bias=False)

    # Modify classifier as in RTInference.py
    num_ftrs = model.classifier[1].in_features
    model.classifier[1] = torch.nn.Sequential(
        torch.nn.Linear(num_ftrs, 512),
        torch.nn.BatchNorm1d(512),
        torch.nn.ReLU(inplace=True),
        torch.nn.Linear(512, 3)
    )

    path = os.path.join(os.path.expanduser('~'), 'model_' + runid + '.pth')
    if not os.path.exists(path):
        print(f"Model file not found at {path}")
        return None
        
    try:
        checkpoint = torch.load(path, map_location='cpu')
        model.load_state_dict(checkpoint)
        model.eval()
        print("Model loaded successfully.")
    except Exception as e:
        print(f"Failed to load model: {e}")
        return None
        
    return model

def check_weights(model):
    print("Checking weights and buffers for NaNs and Infs...")
    has_nan = False
    has_inf = False
    for name, param in model.state_dict().items():
        if torch.isnan(param).any():
            print(f"Tensor {name} has NaNs!")
            has_nan = True
        if torch.isinf(param).any():
            print(f"Tensor {name} has Infs!")
            has_inf = True
            
    if not has_nan and not has_inf:
        print("No NaNs or Infs found in model state_dict.")
    else:
        print("Model state_dict contains NaNs or Infs.")

def run_inference(model):
    print("Running dummy inference with hooks...")
    
    def hook_fn(module, input, output):
        name = module._get_name()
        if torch.isnan(output).any():
            print(f"NaN DETECTED in output of {name}")
        if torch.isinf(output).any():
            print(f"Inf DETECTED in output of {name}")
            
    
    if torch.cuda.is_available():
        print("Moving model to CUDA...")
        model = model.cuda()
    else:
        print("CUDA not available, running on CPU.")

    # Register hooks on all modules
    for name, module in model.named_modules():
        module.register_forward_hook(hook_fn)
        
    # Inspect first layer weights specifically
    first_layer = model.features[0][0]
    print(f"First layer: {first_layer}")
    w = first_layer.weight
    print(f"First layer weight stats: Min={w.min()}, Max={w.max()}, Mean={w.mean()}")
    if torch.isnan(w).any(): print("NaN in first layer weights (double check)")
    if torch.isinf(w).any(): print("Inf in first layer weights (double check)")

    # 1. Random input (using random float 0-255)
    print("Testing random input...")
    dummy_input = torch.rand(1, 1, 224, 224) * 255.0
    if torch.cuda.is_available():
        dummy_input = dummy_input.cuda()
        
    with torch.no_grad():
        output = model(dummy_input)


if __name__ == "__main__":
    runid = '06-04-2024_21-22-37'
    model = load_model(runid)
    if model:
        check_weights(model)
        run_inference(model)
