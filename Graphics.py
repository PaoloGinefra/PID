import pygame
import numpy as np
from pygame.version import ver
from pygame import gfxdraw
from Simulation import Simulation, Circle, Stick, Sensor

class Visual:
    BgColour = (0, 0, 0)
    CircleColour = (68, 135, 242)
    StickColour = (255, 255, 255)
    SensorColour = (50, 168, 82)

    Random = False


    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.isActive = True

        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pass

    def DrawCircle(self, c : Circle, color = CircleColour):
        gfxdraw.circle(self.screen, int(c.position[0]), int(c.position[1]), c.radius, color)

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

    def DrawSensor(self, sensor : Sensor):
        self.DrawCircle(sensor.circle, Visual.SensorColour)

    def Draw(self):
        pygame.display.flip()
        
    def HandleInput(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.isActive = False
            if event.type == pygame.KEYDOWN:
                print('RANDOMIZE')
                Visual.Random = not Visual.Random