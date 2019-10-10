#!/usr/bin/env python
# coding: utf-8

# In[5]:


import numpy as np
from matplotlib import pyplot as plt
from vincenty import vincenty as vc

def xyval(pt):
   ycng = [pt[0], origin[1]]
   xcng = [origin[0], pt[1]]
   x = vc(origin, xcng)
   y = vc(origin, ycng)
   return x, y

origin = [-25,7530198, 28,2277248]
x0, y0 = xyval(origin)

axishift = [-25,7531945, 28,22757296]
xst, yst = xyval(axishift)
theta = np.arctan((yst - y0)/(xst - x0)

point = [-25.75310, 28.22760]
xp, yp = xyval(point)

xpr = round(xp*np.cos(theta) + yp*np.sin(theta), 15)
ypr = round(-xp*np.sin(theta) + yp*np.cos(theta), 15)

heading = np.arctan((ypr - yprlast)/(xpr - xprlast))
heading_gps = heading_gps - theta
xprlast, yprlast = xpr, ypr

plt.plot(x0, y0, 'k.')
plt.plot(xst, yst, 'k.')
plt.plot(xpr, ypr, 'y.')
plt.plot(xh, yh, 'r.')
plt.show()
print(xyval([-25,7531945, 28,22757296]))
print(xyval([-25,7530198, 28,2277248]))

print(heading)


# In[ ]:




