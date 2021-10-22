from numpy.core.fromnumeric import size
from Graphics import Visual
from Simulation import Simulation, Circle, Stick
import numpy as np

vis = Visual(800, 800)

sim = Simulation(Stick(np.array([400, 600]), 600, 20, -179), Circle(25, np.array([500, 100])))
t = 0
while vis.isActive:
    vis.DrawSim(sim)

    for _ in range(10):
        sim.stick.SetAngle(np.sin(t) * 15)
        sim.Step()

        t += Simulation.dt / 2

        sim.PacMan(vis.height)

    vis.HandleInput()