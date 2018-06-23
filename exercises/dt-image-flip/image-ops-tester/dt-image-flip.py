#!/usr/bin/python

import cv2
import numpy as np
import sys
import os
from errno import errorcode
from PIL import Image

try:
    sys.argv[1]
    sys.argv[2]
except:
    print errorcode[99]
    sys.exit(99)

file_path = '/home/nilsiism/duckuments/exercises/dt-image-flip/image-ops-tester/' + sys.argv[1]
file_name = sys.argv[1]
if not os.path.exists(file_path):
    print errorcode[2]
    sys.exit(2)

try:
    Image.open(file_name)
except:
    print errorcode[3]
    sys.exit(3)

img = cv2.imread(sys.argv[1], 1)

dir = '/home/nilsiism/duckuments/exercises/dt-image-flip/image-ops-tester/' + sys.argv[2]
if not os.path.exists(dir):
    os.makedirs(dir)

reg = img
flip = cv2.flip(img, 0, None)
doub = np.concatenate((img,flip),1)

cv2.imwrite(os.path.join(dir , 'regular.jpg'), reg)
cv2.imwrite(os.path.join(dir , 'flip.jpg'), flip)
cv2.imwrite(os.path.join(dir , 'side-by-side.jpg'), doub)
