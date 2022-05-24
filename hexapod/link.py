import numpy as np
from motion.plotter import plot_sphere

class Link:

  def __init__(self, theta, alpha, a, d):
      self.theta = theta
      self.alpha = alpha
      self.a = a
      self.d = d
      self.trans_mat = self.compute_trans_mat(theta)
      
      self.base_pos = None
      self.end_pos = None

  def set_theta(self, theta):
      self.theta = theta
  
  def compute_trans_mat(self, theta):

    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(self.alpha)
    sa = np.cos(self.alpha)

    a = self.a
    d = self.d

    T = np.array([[ct, -st * ca, st * sa, a * ct],
                  [st, ct * ca, -ct * st, a * st],
                  [0, sa, ca, d],
                  [0, 0, 0, 1]])
    
    return T

  def plot(self, ax):
    if self.base_pos is None or self.end_pos is None:
      raise ValueError('Positions not Set')
    
    plot_sphere(self.end_pos, self.a, ax, color='black')
    
    if self.a == 0 and self.d == 0:
      return ax

    pts = np.vstack((self.base_pos, self.end_pos))

    return ax.plot(pts[:,0], pts[:, 1], pts[:,2], color='b', linewidth=3)
