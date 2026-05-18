"""
Quick test of the anomaly detector on a single image.

Usage:
    python test_inference.py path/to/image.png

It will show the original image, anomaly heatmap overlay, and binary mask.
"""

import sys
import cv2
import numpy as np

# Add the parent folder so we can import ssn
sys.path.insert(0, ".")

from anomaly_detector import AnomalyDetector


def main():
    if len(sys.argv) < 2:
        print("Usage: python test_inference.py <image_path>")
        sys.exit(1)

    image_path = sys.argv[1]
    weights_path = "weights/anomaly_model.pt"

    # Load image
    img = cv2.imread(image_path)
    if img is None:
        print(f"Could not load image: {image_path}")
        sys.exit(1)

    # Resize to 512x512 if needed
    img = cv2.resize(img, (512, 512))

    # Init detector
    detector = AnomalyDetector(
        weights_path=weights_path,
        device="cuda",
        score_threshold=0.5,
        mask_threshold=0.5,
    )

    # Run inference
    score, binary_mask, anomaly_map = detector.predict(img)

    print(f"Score: {score:.4f}")
    print(f"Defective: {score > detector.score_threshold}")
    print(f"Anomalous pixels: {np.count_nonzero(binary_mask)} / {512*512}")

    # Visualize
    heatmap = cv2.applyColorMap((anomaly_map * 255).astype(np.uint8), cv2.COLORMAP_JET)
    overlay = cv2.addWeighted(img, 0.6, heatmap, 0.4, 0)

    # Binary mask as 3-channel for display
    mask_vis = cv2.cvtColor(binary_mask, cv2.COLOR_GRAY2BGR)

    # Stack all three side by side
    combined = np.hstack([img, overlay, mask_vis])
    cv2.imshow("Original | Heatmap | Mask", combined)
    print("Press any key to close...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()