import cv2
import numpy as np

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,1920)

fourcc = cv2.VideoWriter_fourcc(*'XVID')

out = cv2.VideoWriter('output.avi',fourcc,30.0,(1920,1080))

cv2.namedWindow("Image",0)

i = 1

while(1):
    ret, frame = cap.read()
    out.write(frame)
    cv2.imshow('Image', frame)
    
    key = cv2.waitKey(1)

    if key == ord ('c'):
        cv2.imwrite(str(i)+".png", frame)
        print(str(i)+".png saved")
        i=i+1
    if key == ord('q'):
        break
        
cap.release()
out.release()
cv2.destroyAllWindows()
