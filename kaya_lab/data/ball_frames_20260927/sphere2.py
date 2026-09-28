import numpy as np, itertools
from scipy.optimize import least_squares
from sphere import contour, rays, r
F=['ball_0p5m.png','ball_10cm_gap.png','ball_touch_centre.png','ball_touch_wheel_a.png','ball_touch_wheel_b.png']
C=[contour(f)[0] for f in F]
def res_one(c,X,Y,h,p):
    R=rays(c,np.radians(p)); Cv=np.array([X,Y,r-h]); d=np.linalg.norm(Cv)
    return np.arccos(np.clip(R@Cv/d,-1,1))-np.arcsin(r/d)
def best_xy(c,h,p):
    g=min(itertools.product(np.arange(0.05,0.8,0.02),np.arange(-0.25,0.25,0.02)),
          key=lambda q:(res_one(c,q[0],q[1],h,p)**2).sum())
    s=least_squares(lambda q:res_one(c,q[0],q[1],h,p),g); return s.x,np.degrees(np.sqrt((s.fun**2).mean()))
# 1) per-frame with the documented camera (h 0.175, pitch 4.89)
print("documented camera h=17.5 cm, pitch 4.89 up:")
for f,c in zip(F,C):
    (X,Y),e=best_xy(c,0.175,4.89); print(f"  {f:20s} X={X*100:5.1f} Y={Y*100:+5.1f} cm  rms {e:.2f} deg")
# 2) joint fit: shared h, p; per-frame X,Y
def jres(q):
    h,p=q[:2]; return np.concatenate([res_one(c,q[2+2*i],q[3+2*i],h,p) for i,c in enumerate(C)])
x0=[0.175,4.89]+[v for c in C for v in best_xy(c,0.175,4.89)[0]]
s=least_squares(jres,x0); h,p=s.x[:2]
print(f"joint fit: h={h*100:.1f} cm pitch={p:.2f} deg up, overall rms {np.degrees(np.sqrt((s.fun**2).mean())):.2f} deg")
for i,f in enumerate(F):
    X,Y=s.x[2+2*i:4+2*i]; e=np.degrees(np.sqrt((res_one(C[i],X,Y,h,p)**2).mean()))
    print(f"  {f:20s} X={X*100:5.1f} Y={Y*100:+5.1f} cm  rms {e:.2f} deg")
