import cv2
img = cv2.imread('label.png',-1)
print(img.dtype)
cv2.imwrite('label1.png',img)
# for i in range(img.shape[0]):
#     for j in range(img.shape[1]):
#       #  print(img[i, j]==[0, 0, 0])
#         if(not (img[i,j]==[0, 0, 0]).all()):
#             if(not (img[i,j]==[0, 128, 0]).all()):
#                 if(not (img[i,j]==[0, 128, 128]).all()):
#                     print(img[i, j])
# see load_label_png.py also.
# import numpy as np
# import PIL.Image
# label_png = 'label.png'
# lbl = np.asarray(PIL.Image.open(label_png))
# for i in range(lbl.shape[0]):
#   for j in range(lbl.shape[1]):
#     print(lbl[i,j])
# print(lbl.dtype)
# print(np.unique(lbl))