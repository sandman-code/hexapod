from mimetypes import init
import numpy as np

import numpy as np

class Leg:

    LEG_COMPONENTS = []
    alpha = 0
    beta = 0
    gamma = 0

    def __init__(self, components, angles) -> None:
        self.LEG_COMPONENTS = components
        self.alpha = angles[0]
        self.beta = angles[1]
        pass

