import numpy as np


class DynamicModel:
    def __init__(self,
                  w: float, 
                  I: float = 0.1939, 
                  x_icr: float = 0.1,
                  m: float = 10.0, 
                  mu_long: float = 0.6, 
                  mu_lat: float = 0.6, 
                  c: float =  0.44, 
                  r: float = 0.0975) -> None:
        self.M = np.array([[m, 0.0], [0, m*x_icr**2 + I]])
        self.C = np.array([[0.0, m*x_icr**2*w], [-m*x_icr**2*w, 0.0]])
        self.B = np.array([[1/r, 1/r], [-c/r, c/r]])

        self.w = w 
        self.I = I 
        self.x_icr = x_icr 
        self.m = m 
        self.c = c 
        self.r = r 
        self.mu_long = mu_long
        self.mu_lat = mu_lat 

    def Frz(self):
        pass 

    def Fry(self):
        pass 

    def Mr(self):
        pass 

    def torque_calculator(self):
        pass