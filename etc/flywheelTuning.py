class Flywheel:
    def __init__(self, stallTorque, stallCurrent, freeRPM, freeCurrent, G, J, maxV = 12, dt = 0.01):
        self.G = G
        self.J = J
        self.R = maxV / stallCurrent
        self.kT = stallTorque / stallCurrent
        self.kV = freeRPM / (maxV - freeCurrent * R)
        self.dt = dt

    def getRPM(self):
        return self.RPM
    
    def step(self, voltage):
        Ax = -(self.G**2 * self.kT) / (self.kV * self.R * self.J) * self.RPM
        Bu = (self.G * self.kT) / (self.R * self.J) * voltage
        dRPM = Ax + Bu
        self.RPM += dRPM * self.dt
        return self.RPM
