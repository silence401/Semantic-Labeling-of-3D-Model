import cv2
import os
lis = os.listdir("./views")
for i in range(len(lis)):
    img_path = "./views/"+os.path.join(lis[i],'label.png')
    img = cv2.imread(img_path, -1)
    print(img.dtype)
    cv2.imwrite('./views/'+os.path.join(lis[i], 'label1.png'), img)