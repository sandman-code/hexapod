from cmath import pi
import matplotlib.pylab as plt
from mpl_toolkits.mplot3d import Axes3D
from hexapod.link import Link

'''

DH Table

Frame | theta | d | a | alpha
=============================
T12   |t1 + 90| 0 |L1 | -90
-----------------------------
T23   | t2    | 0 |L2 | 0
-----------------------------
T34   |t3 - 90| 0 |L3 | -90


'''
fig = plt.figure(figsize=(8, 8))
ax = Axes3D(fig)

L1 = Link(pi / 2, pi / 2, 50, 0)
L2 = Link(0, 0, 100, 0)
L3 = Link(-pi / 2, -pi / 2, 140, 0)

links = [L1, L2, L3]

for l in links:
  l.plot(ax)