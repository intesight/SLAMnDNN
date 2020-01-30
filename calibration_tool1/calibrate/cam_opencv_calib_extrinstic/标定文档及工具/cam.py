import cv2

img = cv2.imread("front.bmp")
print(img.shape)
cv2.namedWindow("Image",0)
cv2.imshow("Image", img)

cv2.waitKey(0)
cv2.destoryAllWindows()
