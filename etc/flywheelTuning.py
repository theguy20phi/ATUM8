import numpy as np

def clamp(num, min, max):
    if num < min: return min
    if num > max: return max
    return num

class Flywheel:
    def __init__(self, stallTorque, stallCurrent, freeRPM, freeCurrent, G, J, maxV = 12, dt = 0.01):
        self.RPM = 0
        self.G = G
        self.J = J
        self.R = maxV / stallCurrent
        self.kT = stallTorque / stallCurrent 
        self.kV = freeRPM / (maxV - freeCurrent * self.R)
        self.dt = dt

    def getRPM(self):
        return self.RPM

    def reset(self):
        self.RPM = 0
    
    def step(self, voltage):
        Ax = -(self.G**2 * self.kT) / (self.kV * self.R * self.J) * self.RPM
        Bu = (self.G * self.kT) / (self.R * self.J) * voltage
        dRPM = Ax + Bu
        self.RPM += dRPM * self.dt
        return self.getRPM()

class PidFF:
    def __init__(self, kP, kI, kD, FF):
        self.I = 0
        self.prevError = 0
        self.output = 0
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.FF = FF
    
    def reset(self):
        self.I = 0
        self.prevState = 0
        self.output = 0
    
    def getOutput(self, state, reference):
        error = reference - state
        P = self.kP * error
        self._updateI(error)
        errorDiff = error - self.prevError
        self.prevError = error
        D = self.kD * errorDiff
        FF = self.FF * reference
        self.output = P + self.I + D + FF
        return self.output
    
    def _updateI(self, error):
        self.I += self.kI * error
        if np.sign(error) != np.sign(self.prevError):
            self.I = 0
        if(self.kI):
            self.I = clamp(self.I, -1 / self.kI, 1 / self.kI)

class EvolutionManager:
    def __init__(self, flywheel, initController, genSize, simTime, kPDev, kIDev, kDDev, FFDev):
        self.flywheel = flywheel
        self.candidate = initController
        self.genSize = genSize
        self.simTime = simTime
        self.kPDev = kPDev
        self.kIDev = kIDev
        self.kDDev = kDDev
        self.FFDev = FFDev
    
    def iterate(self, reference):
        generation = self._createGeneration()
        candidateA = self._getBest(generation, reference)
        generation.remove(candidateA)
        candidateB = self._getBest(generation, reference)
        self.candidate = self._breed(candidateA, candidateB)
        return self.candidate

    def _createGeneration(self):
        generation = [self.candidate]
        for i in range(self.genSize - 1):
            kP = clamp(np.random.default_rng().normal(self.candidate.kP, self.kPDev), 0, np.inf)
            kI = clamp(np.random.default_rng().normal(self.candidate.kI, self.kIDev), 0, np.inf)
            kD = clamp(np.random.default_rng().normal(self.candidate.kD, self.kDDev), 0, np.inf)
            FF = clamp(np.random.default_rng().normal(self.candidate.FF, self.FFDev), 0, np.inf)
            generation += [PidFF(kP, kI, kD, FF)]
        return generation
    
    def _getBest(self, generation, reference):
        evaluations = []
        for controller in generation:
            evaluations += [self._evaluate(controller, reference)]
        return generation[evaluations.index(min(evaluations))]
    
    def _evaluate(self, controller, reference):
        flywheel.reset()
        prevVolt = 0
        zeroCrosses = 0
        totalError = 0
        for i in range(self.simTime):
            volt = controller.getOutput(flywheel.getRPM(), reference) / 1000
            volt = clamp(volt, 0, 12)
            if volt - prevVolt < 0: zeroCrosses += 1
            prevVolt = volt
            totalError += abs(reference - flywheel.getRPM())
            flywheel.step(volt)
        return (zeroCrosses, totalError, abs(reference - flywheel.getRPM()))
    
    def _breed(self, candidateA, candidateB):
        kP = clamp((candidateA.kP + candidateB.kP) / 2, 0, np.inf)
        kI = clamp((candidateA.kI + candidateB.kI) / 2, 0, np.inf)
        kD = clamp((candidateA.kD + candidateB.kD) / 2, 0, np.inf)
        FF = clamp((candidateA.FF + candidateB.FF) / 2, 0, np.inf)
        return PidFF(kP, kI, kD, FF)



flywheel = Flywheel(
    0.24,
    1.7,
    500,
    0.1,
    0.2,
    3e-5
)

target = 2100
dev = 250
evolutionManager = EvolutionManager(flywheel, PidFF(750, 0, 0.1, 0), 10, 3000, 0, 0, 0.001, 0)
candidate = evolutionManager.iterate(target)
for i in range(500):
    reference = np.random.normal(target, dev)
    candidate = evolutionManager.iterate(reference)
    print(f"Iteration #{i}, Reference: #{reference} => kP: {candidate.kP} kI: {candidate.kI}, kD: {candidate.kD}, FF: {candidate.FF}")
flywheel.reset()
for i in range(3000):
    volt = candidate.getOutput(flywheel.getRPM(), 2100) / 1000
    volt = clamp(volt, 0, 12)
    print(f"Voltage: {volt} RPM: {flywheel.getRPM()}")
    flywheel.step(volt)