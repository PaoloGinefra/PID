from numpy.lib.function_base import angle
import pygame
import numpy as np
from pygame.version import ver
from pygame import gfxdraw
from Simulation import Simulation, Circle, Stick

class Visual:
    BgColour = (0, 0, 0)
    CircleColour = (68, 135, 242)
    StickColour = (255, 255, 255)

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.isActive = True

        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pass

    def DrawCircle(self, c : Circle):
        gfxdraw.circle(self.screen, int(c.position[0]), int(c.position[1]), c.radius, Visual.CircleColour)

    def DrawStick(self, s : Stick):
        vert = []
        t = (s.thickness / 2) * s.normal
        vert.append(np.add(s.RightBound, t))
        vert.append(np.add(s.LeftBound, t))
        vert.append(np.add(s.LeftBound, -t))
        vert.append(np.add(s.RightBound, -t))

        gfxdraw.aapolygon(self.screen, tuple(vert), Visual.StickColour)
        return 0

    def DrawSim(self, simulation: Simulation):
        self.screen.fill(Visual.BgColour)

        self.DrawCircle(simulation.circle)
        self.DrawStick(simulation.stick)        

        pygame.display.flip()


    def HandleInput(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.isActive = False