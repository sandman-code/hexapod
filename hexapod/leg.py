from motion.maths import Vector, calc_fk

class Leg:

    def __init__(self, angles, motors, id):
        
        self.id = id

        self.alpha = angles[0]
        self.beta = angles[1]
        self.gamma = angles[2]

        self.alpha_motor = motors[0]
        self.beta_motor = motors[1]
        self.gamma_motor = motors[2]

        self.knee_vector = Vector(0,0,0)
        self.hip_vector = Vector(0,0,0)

        self.ee_pos = Vector(0,0,0)
        
    
    def set_alpha(self, alpha):
        self.alpha = alpha

    def set_beta(self, beta):
        self.beta = beta

    def set_gamma(self, gamma):
        self.gamma = gamma

    def get_alpha(self):
        return self.alpha 

    def get_beta(self):
        return self.beta

    def get_gamma(self):
        return self.gamma

    def move_alpha(self, alpha, time):
        self.alpha = alpha
        self.alpha_motor.moveFromCenter(alpha, time)

    def move_beta(self, beta, time):
        self.beta = beta
        self.beta_motor.moveFromCenter(beta, time)

    def move_gamma(self, gamma, time):
        self.gamma = gamma
        self.gamma_motor.moveFromCenter(gamma, time)

    def update_ee(self):
        pos = calc_fk(self.alpha, self.beta, self.gamma)
        self.ee_pos = Vector(pos[0], pos[1], pos[2])

