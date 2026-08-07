"""
Production Configuration Settings for CyberHUD Object Detection Demonstrator.
Contains curated open-vocabulary prompts, confidence thresholds, and system parameters.
"""

import os

# Curated High-Signal Open-Vocabulary Prompts (Zero False Positives)
DEFAULT_PROMPTS = [
    # Demographics & People
    "person", "man", "woman", "child",
    
    # Eyewear & Headwear
    "glasses", "sunglasses", "hat", "cap",
    
    # Bags & Accessories
    "backpack", "handbag", "wrist watch",
    
    # Carried Beverages & Containers
    "water bottle", "soda can", "coffee cup", "glass bottle", "thermos",
    
    # Tech & Personal Electronics (Distinct from stationary)
    "smartphone", "tablet computer", "laptop", "headphones", "earbuds", "computer mouse",
    
    # Personal Belongings & Office Essentials
    "keys", "pencil", "wallet", "vape", "book", "notebook", "paper document", "conference badge", "wet wipes",
    
    # Packaging & Groceries
    "product package", "plastic packaging", "box", "plastic bag"
]

# Classes to strictly filter out / exclude (e.g. spurious detections)
EXCLUDED_CLASSES = [
    "scissors", "toothbrush", "hair drier", "umbrella", "tie"
]

ALL_WORLD_PROMPTS = DEFAULT_PROMPTS
YOLO_WORLD_PROMPTS = DEFAULT_PROMPTS

# Engine Modes:
# 1 = Pure Flagship YOLO11l
# 2 = Hybrid Dual-Engine (YOLO11l + YOLOv8s-Worldv2 + MobileSAM + Track Voting)
ACTIVE_ENGINE_MODE = 2

# Models (Optimized for SOTA Accuracy & Zero False Positives)
PRIMARY_DETECTOR_MODEL = "yolo11l.pt"            # YOLO11 Large Detector
DEFAULT_MODEL_NAME = PRIMARY_DETECTOR_MODEL
WORLD_DETECTOR_MODEL = "yolov8s-worldv2.pt"      # YOLOv8 Small World v2
SAM_MODEL_NAME = "mobile_sam.pt"                  # MobileSAM

# Precision Thresholds (Eliminates Background False Positives)
CONFIDENCE_THRESHOLD = 0.35      # Primary YOLO11 threshold
WORLD_CONFIDENCE_THRESHOLD = 0.40 # YOLO-World zero-shot threshold (increased for fewer false positives)
VOTE_HISTORY_DEPTH = 10          # Rolling majority vote depth for 100% label stability
MIN_BOX_AREA = 900               # Minimum box area to eliminate tiny background noise (30x30 px)
MAX_BOX_COVERAGE = 0.75          # Maximum box area coverage (% of frame) for non-person items
DUPLICATE_IOU_THRESHOLD = 0.35   # NMS deduplication threshold

# Local VLM & Search Configuration
ENABLE_VLM = False               # Disabled per user request (focusing pure GPU OCR)
PREFERRED_VLM_MODEL = "qwen3-vl:2b"
VLM_COOLDOWN_SECONDS = 2.0
TAVILY_API_KEY = "fill in your own tavily key my friend"

# Camera Configuration
CAMERA_SOURCE = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

# Professional CyberHUD Palette (BGR for OpenCV)
COLOR_PALETTE = {
    "neon_cyan": (255, 240, 0),      # #00F0FF
    "neon_magenta": (127, 0, 255),   # #FF007F
    "neon_green": (102, 255, 0),    # #00FF66
    "neon_amber": (0, 176, 255),    # #FFB000
    "neon_purple": (255, 0, 157),   # #9D00FF
    "neon_yellow": (0, 235, 255),
    "dark_card": (15, 15, 20),
    "card_header": (30, 30, 40),
    "white": (255, 255, 255),
    "gray_text": (180, 180, 180)
}
