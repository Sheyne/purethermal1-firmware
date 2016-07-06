import cv2
from PIL import Image

import argparse

parser = argparse.ArgumentParser(description='Take a picture and display this')
parser.add_argument('index', type=int, help='index of the camera to open')

args = parser.parse_args()


vc = cv2.VideoCapture(args.index)

status, img = vc.read()

if status:
	Image.fromarray(img).show()