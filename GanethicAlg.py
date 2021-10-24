from numpy.random.mtrand import rand
import numpy as np
import random
from Graphics import Visual
from Simulation import Simulation, Circle, Stick, Sensor
from PID_ import PID_controller, Simulator

class Population:
    def __init__(self, size : int) -> None:
        #initialize Parameters
        self.size = size
        self.population = {}
        self.fitnesses = {}
        pass

    def Populate(self):
        #Create new population
        for id in range(self.size):
            self.population[id] = PID_controller.Random(id = id)

        #initialize Fitnesses
        for p in self.population.values():
            self.fitnesses[p.id] = 0

    def Evaluate(self, sim : Simulator):
        #Evaluate all the individual
        for p in self.population.values():
            self.fitnesses[p.id] = sim.Simulate(p)

    def AnalyzeFitnesses(self):
        #statistical analisis of the population
        fitnesses = list(self.fitnesses.values())
        keys = list(self.fitnesses.keys())
        self.best = self.population[keys[np.argmin(fitnesses)]]
        self.best_performance = np.min(fitnesses)
        self.mean_performance = np.mean(fitnesses)
        self.worst_performance = np.max(fitnesses)

    def Kill(self):
        #Kill half of the population 
        for _ in range(self.size // 2):
            p = random.choices(list(self.fitnesses.keys()), weights=list(self.fitnesses.values()), k = 1)[0]
            self.fitnesses.pop(p)
            self.population.pop(p)
    
    def Mix(self, a : PID_controller, b : PID_controller, randomScale : float, id : int):
        def randomCoeff():
            return 1 + random.choice([-1, 1]) * random.random() * randomScale

        #Mixing 2 controllers as a random linear combination + random fluctuation
        t =  self.fitnesses[a.id] / (self.fitnesses[a.id] + self.fitnesses[b.id])

        #P = Simulator.Lerp(a.P, b.P, t) 
        P = random.choice([a.P, b.P])
        P *=  randomCoeff()

        #D = Simulator.Lerp(a.D, b.D, t) 
        D = random.choice([a.P, b.P])
        D *=  randomCoeff()

        #I = Simulator.Lerp(a.I, b.I, t)
        I = random.choice([a.I, b.I])
        I *=  randomCoeff()

        return PID_controller(P, I, D, id)

    def Reproduce(self, gen):
        #Create a copy of the population
        pop = self.population.copy()

        #make the population number even
        if(len(pop) % 2 != 0):
            pop[-1] = PID_controller.Random(scale=100, id = -1)

        #Reproducing
        keys = list(self.population.keys())
        Len = len(keys)
        for p in range(Len):
            B = pop[random.choice(list(pop.keys()))]
            id = p + self.size * (gen + 1)
            self.population[id] = self.Mix(self.population[keys[p]], B, randomScale = 0.5, id = id)
        pass

    def ShowBest(self, sim : Simulator):
        temp = sim.showEvery
        sim.showEvery = 1
        sim.Simulate(self.best)
        sim.showEvery = temp


vis = Visual(800, 800)

st = Stick(np.array([400, 600]), 600, 20, 0, maxTheta = 10, friction_coeff = 1)
cir = Circle(25, np.array([500, 400]))
 
sim = Simulation(st, cir, dt = 0.01)

sens = Sensor(sim, position_coeff = 0.2, radius = 10)

simulator = Simulator(sim, sens, target_distance = 0, n_steps = 5000, visualizer = vis, showEvery = 10000, interpolationFactor = 0.2, randomize = False)

pop = Population(40)
pop.Populate()
gen = 0
pop.Evaluate(simulator)

while vis.isActive:
    print(f'\nGeneration: {gen}')
    pop.AnalyzeFitnesses()
    print(f'Best:{pop.best_performance}, Mean:{pop.mean_performance}, Worst:{pop.worst_performance}')
    print('The best one is: ', end='')
    pop.best.Print()
    pop.ShowBest(simulator)

    pop.Kill()
    pop.Reproduce(gen)

    pop.Evaluate(simulator)

    simulator.randomize = Visual.Random

    gen += 1
    


