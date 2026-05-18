"""
SuperSimpleNet inference wrapper for ROS2.
 
Usage:
    from anomaly_detector import AnomalyDetector
 
    detector = AnomalyDetector("path/to/anomaly_model.pt")
 
    # In your ROS2 callback where you have the 512x512 warped image:
    score, binary_mask, anomaly_map = detector.predict(cv2_bgr_image)
 
    if score > detector.score_threshold:
        # Pile is defective
        # binary_mask is a 512x512 uint8 mask (0 or 255)
        # anomaly_map is a 512x512 float32 heatmap (0.0 to 1.0)
"""
 
import cv2
import numpy as np
import torch
import torch.nn.functional as F
 
# You need the supersimplenet repo on your PYTHONPATH
# e.g.: sys.path.insert(0, "/path/to/supersimplenet")
from model.supersimplenet import SuperSimpleNet 
 
# Same config used during training — must match exactly
TRAIN_CONFIG = {
    "backbone": "wide_resnet50_2",
    "layers": ["layer2", "layer3"],
    "patch_size": 3,
    "noise": True,
    "perlin": True,
    "no_anomaly": "empty",
    "bad": True,
    "overlap": False,
    "adapt_cls_feat": False,
    "noise_std": 0.015,
    "perlin_thr": 0.6,
    "dt": (3, 2),
    "dilate": 7,
    "flips": True,
    "seg_lr": 0.0002,
    "dec_lr": 0.0002,
    "adapt_lr": 0.0001,
    "gamma": 0.4,
    "stop_grad": False,
    "clip_grad": True,
    "epochs": 80,
    "batch": 64,
    "seed": 456654,
    "eval_step_size": 4,
}
 
IMAGE_SIZE = (256, 256)
 
# ImageNet normalization (same as training)
MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)
 
 
class AnomalyDetector:
    def __init__(
        self,
        weights_path: str,
        device: str = None,
        score_threshold: float = 0.5,
        mask_threshold: float = 0.5,
    ):
        """
        Args:
            weights_path: path to the .pt weights file (e.g. "anomaly_model.pt")
            device: "cuda" or "cpu", auto-detected if None
            score_threshold: threshold for image-level anomaly score (above = defective)
            mask_threshold: threshold for pixel-level anomaly map (above = anomalous pixel)
        """
        if device is None:
            self.device = "cuda" if torch.cuda.is_available() else "cpu"
        else:
            self.device = device
 
        self.score_threshold = score_threshold
        self.mask_threshold = mask_threshold
 
        # Build model with same architecture as training
        self.model = SuperSimpleNet(image_size=IMAGE_SIZE, config=TRAIN_CONFIG)
        self.model.load_model(weights_path)
        self.model.to(self.device)
        self.model.eval()
 
        print(f"AnomalyDetector loaded on {self.device}")
 
    def _preprocess(self, bgr_image: np.ndarray) -> torch.Tensor:
        """
        Convert a 512x512 BGR OpenCV image to a normalized tensor.
 
        Args:
            bgr_image: (512, 512, 3) uint8 BGR image from cv2
 
        Returns:
            (1, 3, 512, 512) float32 tensor, ImageNet-normalized
        """
        # BGR -> RGB
        rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
 
        # Resize if needed (should already be 512x512 from homography)
        if rgb.shape[:2] != IMAGE_SIZE:
            rgb = cv2.resize(rgb, (IMAGE_SIZE[1], IMAGE_SIZE[0]))
 
        # Normalize: to float [0,1], then ImageNet normalize
        img = rgb.astype(np.float32) / 255.0
        img = (img - MEAN) / STD
 
        # HWC -> CHW, add batch dim
        tensor = torch.from_numpy(img.transpose(2, 0, 1)).unsqueeze(0)
        return tensor.to(self.device)
 
    @torch.no_grad()
    def predict(self, bgr_image: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
        """
        Run anomaly detection on a single 512x512 BGR image.
 
        Args:
            bgr_image: (512, 512, 3) uint8 BGR image from cv2
 
        Returns:
            score: float, image-level anomaly score (0.0 to 1.0)
            binary_mask: (512, 512) uint8, binary anomaly mask (0 or 255)
            anomaly_map: (512, 512) float32, anomaly heatmap (0.0 to 1.0)
        """
        bgr_image = self._normalize_brightness(bgr_image)
        tensor = self._preprocess(bgr_image)
 
        # Forward pass (inference mode: no mask/label args)
        anomaly_map_raw, score_raw = self.model(tensor)
 
        # Sigmoid to get probabilities
        score = torch.sigmoid(score_raw).item()
        anomaly_map = torch.sigmoid(anomaly_map_raw)
 
        # Resize anomaly map to original image size
        anomaly_map = F.interpolate(
            anomaly_map.unsqueeze(0) if anomaly_map.dim() == 3 else anomaly_map,
            size=IMAGE_SIZE,
            mode="bilinear",
            align_corners=True,
        )
 
        # To numpy
        anomaly_map = anomaly_map.squeeze().cpu().numpy()
 
        # Normalize to [0, 1]
        map_min, map_max = anomaly_map.min(), anomaly_map.max()
        if map_max > map_min:
            anomaly_map = (anomaly_map - map_min) / (map_max - map_min)
 
        # Binary mask by thresholding
        binary_mask = (anomaly_map > self.mask_threshold).astype(np.uint8) * 255
 
        return score, binary_mask, anomaly_map
 
    def is_defective(self, bgr_image: np.ndarray) -> bool:
        """Simple yes/no defect check."""
        score, _, _ = self.predict(bgr_image)
        return score > self.score_threshold

    def _normalize_brightness(self, bgr_image: np.ndarray, target_mean=128) -> np.ndarray:
        lab = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        current_mean = l.mean()
        if current_mean > 0:
            l = np.clip(l.astype(np.float32) * (target_mean / current_mean), 0, 255).astype(np.uint8)
        return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
