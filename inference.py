# inference.py
import torch
from torchvision import transforms
from PIL import Image
from model.model import parsingNet
import numpy as np
import cv2
import os

# config
BACKBONE = '18'
NUM_CLASSES = 5
IMAGE_SIZE = (288, 800)
WEIGHTS = 'weights/tusimple_18.pth'
DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'

# transform
transform = transforms.Compose([
    transforms.Resize(IMAGE_SIZE),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406],
                         [0.229, 0.224, 0.225])
])

# load model
net = parsingNet(BACKBONE, pretrained=False, num_classes=NUM_CLASSES)
state_dict = torch.load(WEIGHTS, map_location=DEVICE)
net.load_state_dict(state_dict['model'], strict=False)
net.to(DEVICE)
net.eval()

def run_inference(image: Image.Image):
    img = transform(image).unsqueeze(0).to(DEVICE)

    with torch.no_grad():
        out = net(img)[0]
        out = out.cpu().numpy().squeeze()

    # binary mask 형태로 변환 (간단화)
    lane_masks = (out.argmax(0) > 0).astype(np.uint8) * 255
    return lane_masks
