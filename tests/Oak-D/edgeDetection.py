import cv2
import numpy as np

path = r"C:\Users\chase\OneDrive\Pictures\Screenshots\Screenshot 2025-07-14 144721.png"
img = cv2.imread(path)
height, width = img.shape[:2]  # works for grayscale or color

# Crop the image
hc = 150
wc = 50
img = img[250:height-hc, wc:width-wc]

cv2.imshow('raw', img)

lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
l, a, b = cv2.split(lab)

# Threshold on L channel to detect dark regions (white=dark pixels)
_, dark_mask = cv2.threshold(l, 60, 255, cv2.THRESH_BINARY_INV)  # tweak threshold


cv2.imshow('darkmask',dark_mask)
# Count white pixels (dark regions) in each column
column_counts = np.sum(dark_mask == 255, axis=0)

# Threshold for minimum pixels per column to keep
min_pixels_per_column = 50

# Create mask for columns to keep
columns_to_keep = column_counts >= min_pixels_per_column

# Make a copy of the image to draw lines on
marked_img = img.copy()

# Thickness of the vertical lines
line_thickness = 4
line_color = (0, 255, 0)  # green BGR

# Draw thick vertical lines on columns that meet threshold
for col_idx, keep in enumerate(columns_to_keep):
    if keep:
        cv2.line(marked_img, (col_idx, 0), (col_idx, marked_img.shape[0]-1), line_color, thickness=line_thickness)

# Show results
cv2.imshow("Marked Columns", marked_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
