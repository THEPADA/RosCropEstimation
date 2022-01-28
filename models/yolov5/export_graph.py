import torch

model = torch.hub.load('ultralytics/yolov5', 'custom', 'best.onnx', encoding="utf8")
traced_graph = torch.jit.trace(model, torch.randn(1, 3, H, W))
traced_graph.save('DeepLab.pth')