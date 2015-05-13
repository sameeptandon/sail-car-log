import numpy as np
import sys
import cv,cv2

if __name__ == '__main__':
    mblyfolder = sys.argv[1]
    ourfolder = sys.argv[2]
    outfolder = sys.argv[3]
    for i in range(3270):
      mblyname = mblyfolder+'/'+str(i+1)+'.png'
      ourname = ourfolder+'/'+str(i+1)+'.png'
      I1 = cv2.imread(mblyname)
      I2 = cv2.imread(ourname)
      cv2.putText(I1, 'Mobileye', (10,20), cv2.FONT_HERSHEY_PLAIN, 1.6, [0,0,255],thickness=1)
      cv2.putText(I2, 'drive.ai', (10,20), cv2.FONT_HERSHEY_PLAIN, 1.6, [0,255,0],thickness=1)
      I = np.concatenate((I1,I2[:,:640,:]), axis=1)
      cv2.imwrite(outfolder+'/'+str(i+1)+'.png',I)
