from numpy.random.mtrand import rand
from PID import *
import numpy as np
import random
from Graphics import Visual
from Simulation import Simulation, Circle, Stick, Sensor
from PID import PID_controller
import pygame

class Population:
    def RandomPID():
        return PID_controller(P = random.random() * 0.01, D = random.random()*0.01, I = random.random()*0.01)

    def __init__(self, size : int) -> None:
        self.size = size
        self.population = []
        self.fitnesses = {}
        pass

    def Populate(self):
        self.population = [Population.RandomPID() for _ in range(self.size)]
        for p in self.population:
            self.fitnesses[p] = 0

    def Evaluate(self, sim : Simulator):
        #print("Starting evaluation:")
        for i, p in enumerate(self.population):
            #print(f"{(i + 1) / self.size * 100} %")
            self.fitnesses[p] = sim.Simulate(p)

        self.best = self.population[np.argmin(list(self.fitnesses.values()))]
        print(np.min(list(self.fitnesses.values())))

    def Kill(self):
        S = np.sum(list(self.fitnesses.values()))
        for _ in range(int(self.size / 2)):
            p = random.choices(self.population, weights=list(self.fitnesses.values()), k = 1)[0]
            
            del self.fitnesses[p]
            self.population.remove(p)
    
    def Mix(a : PID_controller, b : PID_controller, randomScale : float):
        P = Simulator.Lerp(a.P, b.P, random.random()) + random.random() * a.P * randomScale * random.choice([-1, 1])
        D = Simulator.Lerp(a.D, b.D, random.random()) + random.random() * b.D * randomScale * random.choice([-1, 1])
        I = Simulator.Lerp(a.I, b.I, random.random()) + random.random() * b.I * randomScale * random.choice([-1, 1])
        return PID_controller(P, I, D)

    def Reproduce(self):
        pop = [self.population[i] for i in range(len(self.population))]
        if(len(pop) % 2 != 0):
            pop.append(pop[-1])

        for _ in range(int(len(self.population))):
            a = random.randint(0, len(pop)-1)
            A = pop[a]
            del pop[a]

            b = random.randint(0, len(pop)-1)
            B = pop[b]

            self.population.append(Population.Mix(A, B, 1))
        pass

    def ShowBest(self, sim : Simulator):
        self.best.Print()
        sim.showEvery = 1
        sim.Simulate(self.best)
        sim.showEvery = 100000000000000000


vis = Visual(800, 800)

st = Stick(np.array([400, 600]), 600, 20, 0, 30)
cir = Circle(25, np.array([500, 400]))

sim = Simulation(st, cir, dt = 0.01)

sens = Sensor(sim, position_coeff = 0.2, radius = 10)

simulator = Simulator(sim, sens, target_distance = 0, n_steps = 10000, visualizer = vis, showEvery = 10000, interpolationFactor = 0.001, randomize = False)

pop = Population(50)
pop.Populate()
gen = 0
while 1:
    print(gen)
    pop.Evaluate(simulator)
    pop.ShowBest(simulator)
    pop.Kill()
    pop.Reproduce()

    simulator.randomize = Visual.Random

    gen += 1
    


