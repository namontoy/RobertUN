import cv2, numpy as np
# Background cutout (GrabCut) for docs/images/prototype/Prototype_DriverWheels_Bottom.jpg -> driverwheels_bottom_mask.png
import os
S=os.path.dirname(os.path.abspath(__file__))
im=cv2.imread(S+'/../images/prototype/Prototype_DriverWheels_Bottom.jpg')
H,W=im.shape[:2]; f=0.5
sm=cv2.resize(im,None,fx=f,fy=f)
k=1512/1125*f
def P(pts): return (np.array(pts)*k).astype(np.int32)
m=np.full(sm.shape[:2],cv2.GC_BGD,np.uint8)
# probable foreground: generous hull around board + protruding parts
cv2.fillPoly(m,[P([(5,215),(65,215),(65,135),(265,135),(265,215),(1110,215),(1100,1595),(5,1595)])],cv2.GC_PR_FGD)
# sure foreground: board interior and the red terminal
cv2.fillPoly(m,[P([(45,255),(1068,255),(1068,1570),(45,1570)])],cv2.GC_FGD)
cv2.fillPoly(m,[P([(90,160),(240,160),(240,215),(90,215)])],cv2.GC_FGD)
bg=np.zeros((1,65),np.float64); fg=np.zeros((1,65),np.float64)
cv2.grabCut(sm,m,None,bg,fg,6,cv2.GC_INIT_WITH_MASK)
mm=np.where((m==cv2.GC_FGD)|(m==cv2.GC_PR_FGD),255,0).astype(np.uint8)
# keep largest component, fill holes
n,lab,st,_=cv2.connectedComponentsWithStats(mm)
mm=np.where(lab==1+np.argmax(st[1:,cv2.CC_STAT_AREA]),255,0).astype(np.uint8)
cs,_=cv2.findContours(mm,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
mm=np.zeros_like(mm); cv2.drawContours(mm,cs,-1,255,-1)
mm=cv2.morphologyEx(mm,cv2.MORPH_OPEN,np.ones((5,5),np.uint8))
mm=cv2.resize(mm,(W,H),interpolation=cv2.INTER_LINEAR)
mm=cv2.GaussianBlur(mm,(5,5),0)
cv2.imwrite(S+'/driverwheels_bottom_mask.png',mm)
print('fg fraction',round(mm.mean()/255,3))
