# Exact sphere-silhouette fit: ball of radius r resting on the floor at (X fwd, Y left)
# relative to the point below the colour lens. Camera h above floor, pitched up by p.
import cv2, numpy as np, sys
from scipy.optimize import least_squares
fx,fy,cx,cy=404.854,405.079,423.503,237.023; r=0.0765
def contour(f):
    hsv=cv2.cvtColor(cv2.imread(f),cv2.COLOR_BGR2HSV)
    m=cv2.inRange(hsv,(18,90,80),(38,255,255)); m=cv2.morphologyEx(m,cv2.MORPH_CLOSE,np.ones((15,15),np.uint8))
    n,lab,st,_=cv2.connectedComponentsWithStats(m); i=1+np.argmax(st[1:,4]); m=(lab==i).astype(np.uint8)
    cs,_=cv2.findContours(m,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE); c=cs[0][:,0,:].astype(float)
    c=c[(c[:,0]>1)&(c[:,0]<846)&(c[:,1]<478)]   # drop image-border pixels: not true silhouette
    return c, m.sum()
def rays(c,p):
    d=np.stack([(c[:,0]-cx)/fx,(c[:,1]-cy)/fy,np.ones(len(c))],1)  # cam: x right,y down,z fwd
    d/=np.linalg.norm(d,axis=1,keepdims=True)
    # to world (X fwd, Y left, Z up); camera pitched UP by p
    fwd=d[:,2]*np.cos(p)+d[:,1]*np.sin(p); up=d[:,2]*np.sin(p)-d[:,1]*np.cos(p)
    return np.stack([fwd,-d[:,0],up],1)
def fit(f,h,pdeg):
    c,_=contour(f); p=np.radians(pdeg); R=rays(c,p)
    def res(q):
        C=np.array([q[0],q[1],r-h]); dC=np.linalg.norm(C); a=np.arcsin(r/dC)
        return np.arccos(np.clip(R@C/dC,-1,1))-a
    s=least_squares(res,[0.2,0.0]); return s.x, np.degrees(np.sqrt((s.fun**2).mean())), len(c)
for f in sys.argv[1:]:
    for h,p in [(0.175,4.89),(0.175,0.0),(0.175,9.0)]:
        (X,Y),rms,n=fit(f,h,p)
        print(f"{f:22s} pitch {p:4.1f} ball centre X={X*100:5.1f} cm ahead, Y={Y*100:+5.1f} cm (left+) of the colour lens; rms {rms:.2f} deg over {n} edge px")
