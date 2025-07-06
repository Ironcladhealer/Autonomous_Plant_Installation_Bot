import cv2
import numpy as np

# Open the laptop camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Camera not accessible.")
    exit()

# Capture a single frame
ret, img = cap.read()

if not ret:
    print("Error: Failed to capture image.")
    cap.release()
    exit()

# Show the camera window
cv2.imshow("Camera View", img)
cv2.waitKey(0)
cv2.destroyAllWindows()

cap.release()

# Convert to grayscale
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Resize image to a known size for easy division
img = cv2.resize(img, (300, 300))

# Divide the image into a 3x3 grid
height, width = img.shape
box_height, box_width = height // 3, width // 3

matrix = []

for i in range(3):
    row = []
    for j in range(3):
        # Extract each box
        box = img[i * box_height:(i + 1) * box_height, j * box_width:(j + 1) * box_width]
        # Calculate the mean intensity
        mean_intensity = np.mean(box)
        # Threshold: black if mean < 128, white otherwise
        value = 1 if mean_intensity < 128 else 0
        row.append(value)
    matrix.append(row)

# Known patterns (example for 10 patterns)
known_patterns = [
    [1, 0, 1,
     0, 1, 0,
     1, 0, 1],
    [0, 1, 0,
     1, 0, 1,
     0, 1, 0],
    # Add more patterns here...
]

# Flatten the detected matrix
flat_matrix = [cell for row in matrix for cell in row]

# Check which pattern matches the detected matrix
matched_pattern = None
for pattern in known_patterns:
    differences = sum([1 for p, d in zip(pattern, flat_matrix) if p != d])
    if differences <= 1:  # Allow slight variations
        matched_pattern = pattern
        break

# Display the detected matrix
print("Detected Matrix:")
for row in matrix:
    print(row)

# If matched, reconstruct missing values
if matched_pattern:
    reconstructed_matrix = [matched_pattern[i * 3:(i + 1) * 3] for i in range(3)]
    print("Reconstructed Matrix:")
    for row in reconstructed_matrix:
        print(row)
else:
    print("No matching pattern found.")