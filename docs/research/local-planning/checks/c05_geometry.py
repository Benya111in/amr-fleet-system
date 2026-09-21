"""c05: robot 2-circle cover, forklift cover, aisle feasibility (S2/S3), narrow passage tolerance, inflation costs."""
import numpy as np
from math import sqrt, exp, atan2, degrees
# robot 0.60 x 0.40, circles at +-0.15 on the axis
corners = [(0.3,0.2),(0.3,-0.2),(-0.3,0.2),(-0.3,-0.2)]
print("== robot cover radius needed:", max(min(np.hypot(x-cx,y) for cx in (0.15,-0.15)) for x,y in corners))
pts=[(x,y) for x in np.linspace(-0.3,0.3,61) for y in np.linspace(-0.2,0.2,41)]
print("   all rectangle points covered by r=0.25:", all(min(np.hypot(x-0.15,y),np.hypot(x+0.15,y))<=0.25+1e-9 for x,y in pts))
# forklift body 2.0 x 1.0 m, 2 circles at +-0.5 on the axis
fl=[(x,y) for x in np.linspace(-1.0,1.0,81) for y in np.linspace(-0.5,0.5,41)]
r2=max(min(np.hypot(x-0.5,y),np.hypot(x+0.5,y)) for x,y in fl); print("== forklift 2.0x1.0 two-circle radius (+-0.5):", round(r2,3), " one-circle:", round(sqrt(1+0.25),3))
r3=max(min(np.hypot(x-c,y) for c in (-2/3,0,2/3)) for x,y in fl); print("   three-circle radius (0, +-0.667):", round(r3,3))
# S2: 3 m aisle, forklift keeps right: axis 0.8 m from its wall; required centre distance d_c from c02b
# (v2 radii: forklift 3 circles r=0.60, robot 0.25, hard margin 0.30 -> R_hard=1.15; d*=1.26 (tail 0.2 s) .. 1.60 (tail 1.0 s))
W=3.0
for d_c in (1.15, 1.26, 1.60):
    rob_axis = 0.8 + d_c; edge_wall = W - (rob_axis + 0.2); edge_fl = d_c - 0.2 - 0.5
    print("== S2 (3 m aisle, forklift axis 0.8 m from wall) d_c=%.2f: robot axis %.2f m from forklift wall, own-wall edge gap %.2f, edge gap to forklift %.2f, lateral deviation from aisle centre %.2f"%(d_c,rob_axis,edge_wall,edge_fl,rob_axis-W/2))
print("   forklift centred (axis 1.5): max d_c with 0.1 m wall gap = %.2f -> edge gap to forklift %.2f (d_c 1.20 < required 1.26 even for the shortest brake tail -> hard check forces yield)"%(W-0.1-0.2-1.5, W-0.1-0.2-1.5-0.7))
print("   1-circle forklift r=1.2 (brief v1): free lateral = 3.0 - 2.4 = 0.6 m < robot 0.4 + margins -> infeasible")
# S3: person at y=-0.7 walking along a 3 m aisle; required centre distance (person, delta 0.05, tail<=0.9 s): ~1.1
for yp in (-0.7, 0.0):
    need = yp + 1.1; print("== S3 person at y=%.1f: robot axis >= %.2f -> deviation %.2f m, own-wall edge gap %.2f"%(yp, need, need, 1.5-need-0.2))
# narrow passage 0.6 m: centred robot, wall distance from centre 0.30 m; inflation cost with r_ins=0.20, k_s=3.0
k=3.0; rins=0.20; rcirc=sqrt(0.3**2+0.2**2)
c=lambda d: 252*exp(-k*(d-rins)) if d>rins else 253
print("== inflation: r_circ=%.3f, c(r_circ)=%.0f, c(0.30)=%.0f (passage centre), c(0.25)=%.0f"%(rcirc,c(rcirc),c(0.30),c(0.25)))
# heading tolerance in 0.6 m passage: half-width extent 0.3 sin psi + 0.2 cos psi <= 0.3 - lateral offset
for off in (0.0, 0.02, 0.05):
    psis=np.radians(np.linspace(0,30,30001)); ext=0.3*np.sin(psis)+0.2*np.cos(psis)
    ok=psis[ext<=0.3-off]; print("   passage lateral offset %.2f m -> max heading error %.1f deg"%(off, degrees(ok.max())))
