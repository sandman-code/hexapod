import sys
import matplotlib.pyplot as plt
import numpy as np
from controller import *
sys.path.append('../hexapod')

import hexapodcore

hexapod = Hexapod()

walk(hexapod, 0.03)


