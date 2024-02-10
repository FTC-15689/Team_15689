import PIL.Image as Image
import cv2
import matplotlib.pyplot as plt
import numpy as np
import os

# clear the debug folder
for file in os.listdir("debug"):
    os.remove(os.path.join("debug", file))

img = Image.open(os.path.join(os.getcwd(), "input.jpg"))
img = np.array(img)
plt.imsave("debug/0.png", img)

# get the median of the top 25% of the red and blue channel
red = np.median(np.sort(img[:, :, 0].flatten())[-int(0.25 * img[:, :, 0].size):])
blue = np.median(np.sort(img[:, :, 2].flatten())[-int(0.25 * img[:, :, 2].size):])

luminosity = img[:, :, (2 * (blue > red))]
plt.imsave("debug/1.png", luminosity, cmap="gray")

# find the difference between the luminosity and the average of the green and blue channel
# diff_lum = img[:, :, 0] - np.mean(img[:, :, 1:], axis=2)
diff_lum = luminosity - np.mean(img[:, :, 1:], axis=2)
diff_lum = diff_lum.clip(0, 255)
diff_lum = cv2.threshold(diff_lum, 50, 255, cv2.THRESH_BINARY)[1]
diff_lum = cv2.GaussianBlur(diff_lum, (5, 5), 0)
plt.imsave("debug/2.png", diff_lum, cmap="gray")

# find the edges
diff_lum = diff_lum.astype(np.uint8)
edges = cv2.Canny(diff_lum, 100, 200)
edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
plt.imsave("debug/3.png", edges, cmap="gray")

# find the contours
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours = sorted(contours, key=cv2.contourArea, reverse=True)
plt.imsave("debug/4.png", cv2.drawContours(img.copy(), contours, -1, (0, 255, 0), 3))

# only use the 5 largest contours
contours = contours[:5]

# group the contours by the center of the bounding box
left = []
center = []
right = []

leftX = img.shape[1] / 3
rightX = img.shape[1] * 2 / 3
centerX = img.shape[1] / 2

for c in contours:
    minRect = cv2.minAreaRect(c)
    box = cv2.boxPoints(minRect)
    box = np.intp(box)

    centerP = (box[0] + box[1] + box[2] + box[3]) / 4
    if centerP[0] < leftX:
        left.append(c)
    elif centerP[0] > rightX:
        right.append(c)
    else:
        center.append(c)

# find the group with the highest sum of areas
leftA = np.sum([cv2.contourArea(c) for c in left])
centerA = np.sum([cv2.contourArea(c) for c in center])
rightA = np.sum([cv2.contourArea(c) for c in right])

print(f'Left: {leftA}, Center: {centerA}, Right: {rightA}')

tmp = img.copy()
if leftA > centerA and leftA > rightA:
    print('Left')
    cv2.drawContours(tmp, left, -1, (0, 255, 0), -1)
    bbox = cv2.boundingRect(np.concatenate(left))
    cv2.rectangle(tmp, (bbox[0], bbox[1]), (bbox[0] + bbox[2], bbox[1] + bbox[3]), (0, 0, 255), 5)
elif centerA > leftA and centerA > rightA:
    print('Center')
    cv2.drawContours(tmp, center, -1, (0, 255, 0), -1)
    bbox = cv2.boundingRect(np.concatenate(center))
    cv2.rectangle(tmp, (bbox[0], bbox[1]), (bbox[0] + bbox[2], bbox[1] + bbox[3]), (0, 0, 255), 5)
else:
    print('Right')
    cv2.drawContours(tmp, right, -1, (0, 255, 0), -1)
    bbox = cv2.boundingRect(np.concatenate(right))
    cv2.rectangle(tmp, (bbox[0], bbox[1]), (bbox[0] + bbox[2], bbox[1] + bbox[3]), (0, 0, 255), 5)

plt.imsave("debug/5.png", tmp)
