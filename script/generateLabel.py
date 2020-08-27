import cv2
import numpy as np
img = np.zeros((256,256,3))
color = np.array([[255, 255, 0], [0, 255, 0], [0, 255, 255], [255, 0, 255]])
for h in range(4):
    for n in range(50):
        for i in range(256):
            img[255-h*50-n,i] = np.array(color[h])
cv2.imwrite('label.png',img)
