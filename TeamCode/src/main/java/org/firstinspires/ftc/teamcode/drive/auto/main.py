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
# limit to values between 200 and 255
pixels = ()
for i in range(180, 255):
    tmp = luminosity.clip(i, 255)
    tmp = cv2.GaussianBlur(tmp, (5, 5), 0)
    pixels += (np.sum(tmp > i),)

pixels = np.array(pixels)

deriv2 = np.diff(pixels, 2)

# find the maximum of the second derivative
max_index = np.argmax(deriv2)
max_value = pixels[max_index]

# find the index of the maximum value
max_index = np.argmax(pixels > max_value)
max_index = np.clip((max_index + 180) * 1.2, 180, 255)
print(f'Optimal value: {max_index}')

luminosity = luminosity.clip(max_index, 255)
luminosity = (luminosity - max_index) * 255 / (255 - max_index)
luminosity = cv2.GaussianBlur(luminosity, (15, 15), 0)
luminosity = np.array(luminosity, dtype=np.uint8)
plt.imsave("debug/2.png", luminosity, cmap="gray")

# find the edges
grad_x, grad_y = np.gradient(luminosity)
grad = np.sqrt(grad_x ** 2 + grad_y ** 2)
grad = grad / grad.max() * 255
grad = np.array(grad, dtype=np.uint8)
plt.imsave("debug/3.png", grad, cmap="gray")

edges = cv2.Canny(grad, 100, 200)
edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
plt.imsave("debug/4.png", edges, cmap="gray")

# find the contours
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours = sorted(contours, key=cv2.contourArea, reverse=True)

# remove contours whose area is less than 0.01% of the total area
contours = [c for c in contours if cv2.contourArea(c) > 0.0001 * img.size]

tmp = img.copy()
cv2.drawContours(tmp, contours, -1, (0, 255, 0), 3)
plt.imsave("debug/5.png", tmp)

areas = [cv2.contourArea(c) for c in contours]

areas = np.array(areas) / np.sum(areas)

# remove areas under 5%
contours = [c for i, c in enumerate(contours) if areas[i] > 0.05]
print(f'Found {len(contours)} contours')

tmp = img.copy()
cv2.drawContours(tmp, contours, -1, (0, 255, 0), 3)
for c in contours:
    minRect = cv2.minAreaRect(c)
    box = cv2.boxPoints(minRect)
    box = np.intp(box)
    cv2.drawContours(tmp, [box], 0, (0, 0, 255), 2)

plt.imsave("debug/6.png", tmp)

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
    cv2.drawContours(tmp, left, -1, (0, 255, 0), 5)
elif centerA > leftA and centerA > rightA:
    print('Center')
    cv2.drawContours(tmp, center, -1, (0, 255, 0), 5)
else:
    print('Right')
    cv2.drawContours(tmp, right, -1, (0, 255, 0), 5)

plt.imsave("debug/7.png", tmp)
